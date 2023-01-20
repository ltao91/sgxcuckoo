/*
 * Copyright (C) 2011-2021 Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "Enclave.h"
#include "Enclave_t.h" /* print_string */
#include <stdarg.h>
#include <stdio.h> /* vsnprintf */
#include <string.h>
#include <sgx_trts.h>
#include <string>
#include "common/hash.h"
#include <iostream>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <cassert>
#include <unordered_map>

using namespace std;

/*
 * printf:
 *   Invokes OCALL to display the enclave buffer to the terminal.
 */
int printf(const char *fmt, ...)
{
    char buf[BUFSIZ] = {'\0'};
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, BUFSIZ, fmt, ap);
    va_end(ap);
    ocall_print_string(buf);
    return (int)strnlen(buf, BUFSIZ - 1) + 1;
}

uint32_t hoge_hash()
{
    std::string s = "hello";
    uint32_t h1 = 0, h2 = 0;
    hashlittle2(s.c_str(), s.length(), &h1, &h2);
    return h1;
}

class OptCuckoo
{
public:
    class Data
    {
    public:
        string key;
        int val;
        Data()
        {
        }
        Data(string t_key, int t_val)
        {
            key = t_key;
            val = t_val;
        }
    };
    class Node
    {
    public:
        unsigned char tag; // why? how many bits?
        Data *data;
        Node()
        {
        }
        Node(unsigned char t_tag, string t_key, int t_val)
        {
            tag = t_tag;
            data = new Data(t_key, t_val);
        }
        ~Node(){
            if(data!=NULL)delete data;
        }
    };

    const int SLOTS_NUM = 4;
    const int MAX_LOOP_FOR_PUT = 80 * 1000;

    int table_size;
    int key_size;
    int key_versions_size;

    vector<vector<Node *>> table;
    vector<vector<mutex>> table_locks;
    vector<vector<int>> visited;
    vector<vector<int>> key_versions;
    vector<vector<mutex>> key_versions_locks;

    // mutex giant_write_lock;

    unsigned char get_tag(const uint32_t input)
    {
        uint32_t t = input & 0xff;
        return (unsigned char)t + (t == 0);
    }

    void ABORT()
    {
        // this is for debug and for annotation
        // cout<<"ABORTED"<<endl;
    }
    OptCuckoo(int t_table_size)
    {
        table_size = t_table_size;
        table = vector<vector<Node *>>(table_size, vector<Node *>(SLOTS_NUM));
        // visited = vector<vector<int>>(table_size, vector<int>(SLOTS_NUM));
        key_versions = vector<vector<int>>(table_size, vector<int>(SLOTS_NUM));
        for (int i = 0; i < table_size; i++)
        {
            key_versions_locks.emplace_back(vector<mutex>(SLOTS_NUM));
            table_locks.emplace_back(vector<mutex>(SLOTS_NUM));
        }

        key_versions_size = table_size;
        // longest = vector<pair<pair<int, int>, int>>();
    }

    ~OptCuckoo(){
        for(int i=0;i<table_size;i++){
            for(int j=0;j<SLOTS_NUM;j++){
                if(table[i][j]!=NULL){
                    delete table[i][j];
                }
            }
        }
    }

    double get_version_t = 0;

    int get_version(int l, int r)
    {
        return __sync_add_and_fetch(&key_versions[l][r], 0);
    }

    double increase_version_t = 0;
    void increase_version(int l, int r)
    {
        __sync_add_and_fetch(&key_versions[l][r], 1);
    }

    int get(std::string key)
    {
        uint32_t h1 = 0, h2 = 0;
        hashlittle2(key.c_str(), key.length(), &h1, &h2);
        unsigned char tag = get_tag(h1);
        if (h1 < 0)
            h1 += table_size;
        if (h2 < 0)
            h2 += table_size;
        h1 = (h1) % table_size;
        h2 = (h2) % table_size;

        // let's try to get
        while (true)
        {
            for (int i = 0; i < SLOTS_NUM; i++)
            {
                Node *node = table[h1][i];
                uint32_t start_version = get_version(h1, i);
                if (node != NULL && node->tag == tag && node->data != NULL)
                {
                    Data *data = node->data;
                    if (key == data->key)
                    {
                        int val = data->val;
                        uint32_t end_version = get_version(h1, i);
                        if (start_version != end_version || start_version & 0x1)
                        {
                            ABORT();
                            continue;
                        }
                        return val;
                    }
                }
            }

            for (int i = 0; i < SLOTS_NUM; i++)
            {
                Node *node = table[h2][i];
                uint32_t start_version = get_version(h2, i);
                if (node != NULL && node->tag == tag && node->data != NULL)
                {
                    Data *data = node->data;
                    if (key == data->key)
                    {
                        int val = data->val;
                        uint32_t end_version = get_version(h2, i);
                        if (start_version != end_version || start_version & 0x1)
                        {
                            ABORT();
                            continue;
                        }
                        return val;
                    }
                }
            }

            // not found
            int temp_val;
            return temp_val;
        }
    }

    int aborted_num = 0;
    int validation_fail = 0;
    double sum_time = 0;
    int evict_null = 0;
    vector<pair<pair<int, int>, int>> longest;

    void put(std::string key, int val, int TID)
    {
        int i = 0;
        while (!put_impl(key, val, TID))
        {
            ABORT();
            i++;
            aborted_num++;
            if (i >= 100000)
            {
                return;
            }
        }
    }
    double write_lock_time = 0;
    double hash_time = 0;
    double w_1 = 0, w_2 = 0, w_3 = 0;
    int c1 = 0, c2 = 0, c3 = 0;

    bool put_impl(std::string key, int val, int TID)
    {
        string original_key = key;
        int original_val = val;
        unsigned char original_tag;

        vector<pair<uint32_t, int>> path;
        vector<int> path_versions_history;

        queue<pair<vector<pair<uint32_t, int>>, vector<int>>> que; //(path,version)
        bool found_shortest_path = false;

        uint32_t th1 = 0;
        uint32_t th2 = 0;
        hashlittle2(key.c_str(), key.length(), &th1, &th2);
        unsigned char tag = get_tag(th1);
        original_tag = tag;
        auto original_node = new Node(original_tag, original_key, original_val);
        if (th1 < 0)
            th1 += table_size;
        if (th2 < 0)
            th2 += table_size;
        th1 = (th1) % table_size;
        th2 = (th2) % table_size;

        // search in h1
        for (int i = 0; i < SLOTS_NUM; i++)
        {
            Node *node = table[th1][i];
            if (node == nullptr)
            {
                pair<int, int> index = make_pair(th1, i);
                table_locks[index.first][index.second].lock();
                if (table[index.first][index.second] != nullptr)
                {
                    ABORT();
                    table_locks[index.first][index.second].unlock();
                    continue;
                }
                increase_version(index.first, index.second);
                table[index.first][index.second] = original_node;
                increase_version(index.first, index.second);
                table_locks[index.first][index.second].unlock();
                return true;
            }
            else if (node->tag == tag && node->data->key == key)
            {
                table_locks[th1][i].lock();
                if (node->data->key != key)
                {
                    table_locks[th1][i].unlock();
                    continue;
                }
                increase_version(th1, i);
                node->data->val = val;
                increase_version(th1, i);
                table_locks[th1][i].unlock();
                return true;
            }
        }
        // search in h2
        for (int i = 0; i < SLOTS_NUM; i++)
        {
            Node *node = table[th2][i];
            if (node == nullptr)
            {
                pair<int, int> index = make_pair(th2, i);
                table_locks[index.first][index.second].lock();
                if (table[index.first][index.second] != nullptr)
                {
                    ABORT();
                    table_locks[index.first][index.second].unlock();
                    continue;
                }
                increase_version(index.first, index.second);
                table[index.first][index.second] = original_node;
                increase_version(index.first, index.second);
                table_locks[index.first][index.second].unlock();
                return true;
            }
            else if (node->tag == tag && node->data->key == key)
            {
                table_locks[th2][i].lock();
                if (node->data->key != key)
                {
                    table_locks[th2][i].unlock();
                    continue;
                }
                increase_version(th2, i);
                node->data->val = val;
                increase_version(th2, i);
                table_locks[th2][i].unlock();
                return true;
            }
        }

        // if both of them are full, do bfs
        for (int i = 0; i < SLOTS_NUM; i++)
        {
            {
                vector<pair<uint32_t, int>> tp;
                tp.push_back(make_pair(th1, i));
                vector<int> tv;
                que.push(make_pair(tp, tv));
            }
            {
                vector<pair<uint32_t, int>> tp;
                tp.push_back(make_pair(th2, i));
                vector<int> tv;
                que.push(make_pair(tp, tv));
            }
        }

        auto add_next_node = [&](vector<pair<uint32_t, int>> &p, vector<int> &v)
        {
            assert(v.size() + 1 == p.size());
            bool is_success = false;
            pair<int, int> before = p[p.size() - 1];
            assert(table[before.first][before.second] != NULL);
            v.push_back(get_version(before.first, before.second));
            string key = table[before.first][before.second]->data->key;
            int val = table[before.first][before.second]->data->val;
            uint32_t h1 = 0, h2 = 0;
            hashlittle2(key.c_str(), key.length(), &h1, &h2);
            unsigned char tag = get_tag(h1);
            if (h1 < 0)
                h1 += table_size;
            if (h2 < 0)
                h2 += table_size;
            h1 = (h1) % table_size;
            h2 = (h2) % table_size;

            // search in h1
            for (int i = 0; i < SLOTS_NUM; i++)
            {
                auto now = get_version(h1, i);
                Node *node = table[h1][i];
                if (node == NULL)
                {
                    v.push_back(now);
                    p.push_back(make_pair(h1, i));
                    is_success = true;
                    found_shortest_path = true;
                    break;
                }
            }
            if (is_success)
            {
                return;
            }

            // search in h2
            for (int i = 0; i < SLOTS_NUM; i++)
            {
                auto now = get_version(h2, i);
                Node *node = table[h2][i];
                if (node == NULL)
                {
                    v.push_back(now);
                    p.push_back(make_pair(h2, i));
                    is_success = true;
                    found_shortest_path = true;
                    break;
                }
            }
            if (is_success)
            {
                return;
            }

            // if both of them are full, choose node to evict
            Node *evict_node = NULL;
            for (int i = 0; i < SLOTS_NUM; i++)
            {
                if (find(p.begin(), p.end(), make_pair(h1, i)) == p.end())
                {
                    auto np = p;
                    np.push_back(make_pair(h1, i));
                    que.push(make_pair(np, v));
                }
                if (find(p.begin(), p.end(), make_pair(h2, i)) == p.end())
                {
                    auto np = p;
                    np.push_back(make_pair(h2, i));
                    que.push(make_pair(np, v));
                }
            }
        };
        int counter = 0;
        while (!que.empty())
        {
            auto now = que.front();
            que.pop();
            add_next_node(now.first, now.second);
            if (found_shortest_path)
            {
                path = now.first;
                path_versions_history = now.second;
                break;
            }
            counter++;
            // cout<<counter<<endl;
        }

        if (!found_shortest_path)
        {
            ABORT();
            return false;
        }
        assert(path.size() != 1);
        // if (path.size() == 1)
        // {
        //     pair<int, int> index = path.front();
        //     table_locks[index.first][index.second].lock();
        //     if (table[index.first][index.second] != NULL)
        //     {
        //         ABORT();
        //         table_locks[index.first][index.second].unlock();
        //         return false;
        //     }
        //     increase_version(index.first, index.second);
        //     table[index.first][index.second] = original_node;
        //     increase_version(index.first, index.second);
        //     table_locks[index.first][index.second].unlock();
        //     return true;
        // }
        for (int i = path.size() - 1; i > 0; i--)
        {
            auto to = path[i];
            auto from = path[i - 1];

            assert((table[to.first][to.second] != NULL || i == path.size() - 1) && table[from.first][from.second] != NULL);

            // lock smaller in first.
            if (to < from)
            {
                table_locks[to.first][to.second].lock();
                table_locks[from.first][from.second].lock();
            }
            else
            {
                table_locks[from.first][from.second].lock();
                table_locks[to.first][to.second].lock();
            }

            if (get_version(to.first, to.second) != path_versions_history[i] || path_versions_history[i] & 0x1)
            {
                ABORT();
                table_locks[to.first][to.second].unlock();
                table_locks[from.first][from.second].unlock();
                return false;
            }
            if (get_version(from.first, from.second) != path_versions_history[i - 1] || path_versions_history[i - 1] & 0x1)
            {
                ABORT();
                table_locks[to.first][to.second].unlock();
                table_locks[from.first][from.second].unlock();
                return false;
            }
            increase_version(to.first, to.second);
            table[to.first][to.second] = table[from.first][from.second];
            // table[from.first][from.second] = NULL;
            increase_version(to.first, to.second);
            table_locks[to.first][to.second].unlock();
            table_locks[from.first][from.second].unlock();
        }
        table_locks[path[0].first][path[0].second].lock();
        if (get_version(path[0].first, path[0].second) != path_versions_history[0] || path_versions_history[0] & 0x1)
        {
            ABORT();
            table_locks[path[0].first][path[0].second].unlock();
            return false;
        }
        increase_version(path[0].first, path[0].second);
        table[path[0].first][path[0].second] = original_node;
        increase_version(path[0].first, path[0].second);
        table_locks[path[0].first][path[0].second].unlock();
        return true;
    }
    // OptCuckoo(const OptCuckoo &cuckoo)
    // {
    // }
};

OptCuckoo *cuckoo;

void ecall_init(){
    cuckoo = new OptCuckoo(8000*1000);
}

void ecall_put(int n, int tid)
{
    std::string s = "random" + to_string(n);
    cuckoo->put(s,n,tid);
}
