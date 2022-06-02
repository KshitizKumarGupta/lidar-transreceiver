#include <iostream>
#include <fstream>
#include <algorithm>
#include <deque>
#include <chrono>
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#define LENGTH 1448
#define HEADER_LENGTH 256
#define MAX_FILE_SIZE 2000000

using namespace std;

deque<string> split (string s, string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    string token;
    deque<string> res;

    while ((pos_end = s.find (delimiter, pos_start)) != string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}

class FileMD
{
public:
    std::string name;
    long int size;
    std::string path;
    std::string dir_path;

    FileMD()
    {
        // empty constructor
        // char buffer[HEADER_LENGTH];
        // getcwd(buffer, HEADER_LENGTH);
        // this->dir_path = buffer;
        // this->dir_path += "/recv_folder";
        this->dir_path = "/home/untitlederror-09/Downloads/recv_folder";
    }

    FileMD(std::string path)
    {
        this->path = path;
        this->name = path.substr(path.find_last_of('/'), (path.size() - path.find_last_of('/')));
        this->dir_path = path.substr(path.find_last_of('/'), path.find_last_of('/'));

        FILE *fp = fopen(path.c_str(), "rb");
        fseek(fp, 0, SEEK_END);
        this->size = ftell(fp);
        fclose(fp);
    }

    void setName(std::string name){
        this->name = name;
        this->path = this->dir_path+this->name;
    }
};

class DirAccess
{
private:
    std::string dir_path;

public:
    DirAccess(std::string dir_path)
    {
        this->dir_path = dir_path;
    }

    deque<std::string> *get_files()
    {
        struct dirent **namelist;
        int i, n;
        deque<std::string> *list = new deque<std::string>();

        n = scandir(this->dir_path.c_str(), &namelist, 0, alphasort);
        if (n < 0)
            perror("[Error] : Directory scan failed\n");
        else
        {
            for (i = 2; i < n; i++)
            {
                // printf("./send_folder/%s\n", namelist[i]->d_name);

                list->push_back(namelist[i]->d_name);
                free(namelist[i]);
            }
        }
        free(namelist);

        return list;
    }
};

class Client_socket
{
    int PORT;
    std::string IP;

    int server_socket;

    struct sockaddr_in address;
    int address_length;

public:
    Client_socket(int PORT, std::string IP)
    {
        // close_socket();
        create_socket();
        this->PORT = PORT;
        this->IP = IP;

        address.sin_family = AF_INET;
        address.sin_port = htons(this->PORT);
        address_length = sizeof(address);
        if (inet_pton(AF_INET, this->IP.c_str(), &address.sin_addr) <= 0)
        {
            cout << "[ERROR] : Invalid address\n";
        }

        create_connection();
    }

    void create_socket()
    {
        if ((server_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
            perror("[ERROR] : Socket failed.\n");
            exit(EXIT_FAILURE);
        }
        cout << "[LOG] : Socket Created Successfully.\n";
    }

    void create_connection()
    {
        if (connect(server_socket, (struct sockaddr *)&address, sizeof(address)) < 0)
        {
            perror("[ERROR] : connection attempt failed.\n");
            exit(EXIT_FAILURE);
        }
        cout << "[LOG] : Connection Successfull.\n";
    }

    std::string receive_file()
    {
        char buffer[LENGTH] = {};
        bzero(buffer, LENGTH);
        ssize_t content_length = 0;
        int recv_block = 0;
        char temp[4], file_name[128];
        deque<std::string> headers;
        FileMD *file = new FileMD();
        // void* recv_data = malloc(MAX_FILE_SIZE);
        ssize_t mem_content_length = 0;

        do
        {
            try
            {
                // Recieving Headers = char[128]+ld[32]
                // cout << "[LOG] : Waiting for Ping...\n";
                read(server_socket, buffer, HEADER_LENGTH);
                headers = split(buffer, "<^v>");
                // cout<<"[DEBUG] : "<<buffer<<endl;
            }
            catch (exception e)
            {
                cout << "[LOG] : Invalid transmission detected. Resuming watch...\n";
            }
        } while (strcmp(headers[0].c_str(), "PING"));

        file->size = atoi(headers[1].c_str());
        file->setName(headers[2]);

        // cout << "[LOG] : Sending Acknowledgement...\n";
        if (send(server_socket, buffer, HEADER_LENGTH, 0) <= 0)
        {
            perror("[Error] : Failed to send Acknowledgement\n");
            return to_string(0);
        }

        // cout << "[LOG] : Acknowledgement Sent. Waiting for File \"" << file->name << "\" of size " << file->size << endl;

        // cout<<"[DEBUG] : "<<file->path<<endl;
        FILE *file_pointer = fopen(file->path.c_str(), "wb");

        if (file_pointer != NULL)
        {
            // cout << "[LOG] : File is ready to be Received.\n";
        }
        else
        {
            cout << "[ERROR] : File loading failed, Exiting.\n";
            exit(EXIT_FAILURE);
        }
        
        char recv_data[file->size];
        // std::string recv_data;
        // bzero(recv_data, MAX_FILE_SIZE);
        while (((recv_block = read(server_socket, buffer, LENGTH)) > 0) || (content_length < file->size))
        {
            auto start = std::chrono::high_resolution_clock::now();
            content_length += recv_block;
            if ((recv_block == LENGTH) || (content_length == file->size))
            {
                fwrite(buffer, sizeof(char), recv_block, file_pointer);
                // memcpy(recv_data+mem_content_length, buffer, recv_block);
                // recv_data+=buffer;
                // sprintf(recv_data, "%s%s", recv_data, buffer);
            }
            else
            {
                cout << "[ERROR] : Frame error, Exititng.\nCurrent:"<<recv_block<<endl<<(file->size == content_length)<<endl;
            auto finish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = finish - start;
            std::cout << "Elapsed time: " << elapsed.count() << " s\n";
                break;
            }
            mem_content_length += recv_block;
            // cout<<content_length<<", mem:"<<mem_content_length<<endl;
            bzero(buffer, LENGTH);
            if (file->size == content_length)
            {
                // cout << "[LOG] : Total Received\n";
                break;
            }
            // auto finish = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double> elapsed = finish - start;
            // std::cout << "Elapsed time: " << elapsed.count() << " s\n";
        }
        // cout << "[LOG] : Data received " << content_length << " bytes\n";
        // cout << "[LOG] : Saving data to file_pointer.\n";

        // fwrite(recv_data, sizeof(char), content_length, file_pointer);
        // fwrite(recv_data.c_str(), sizeof(char), content_length, file_pointer);

        // cout << "[LOG] : File Saved.\n";

        sprintf(buffer, "%ld", content_length);

        // cout << "[LOG] : Sending Post-Transfer Acknowledgement...\n";
        if (send(server_socket, buffer, HEADER_LENGTH, 0) <= 0)
        {
            perror("[Error] : Failed to send Acknowledgement\n");
            return to_string(0);
        }

        fclose(file_pointer);
        // free(recv_data);

        std::string file_path = file->path;
        delete(file);

        return file_path;
    }

    void receieve_folder_files(){

    }

    void close_socket()
    {
        // fflush(server_socket);
        close(server_socket);
    }

    ~Client_socket()
    {
        close_socket();
    }
};