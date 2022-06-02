#include <iostream>
#include <fstream>
#include <algorithm>
#include <deque>
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#define LENGTH 1400
#define HEADER_LENGTH 256

using namespace std;

deque<string> split(string s, string delimiter)
{
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    string token;
    deque<string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != string::npos)
    {
        token = s.substr(pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back(token);
    }

    res.push_back(s.substr(pos_start));
    return res;
}

class FileMD
{
public:
    std::__cxx11::string name;
    long int size;
    std::__cxx11::string path;

    FileMD()
    {
        // empty constructor
    }

    FileMD(std::__cxx11::string path)
    {
        this->path = path;
        this->name = path.substr(path.find_last_of('/'), (path.size() - path.find_last_of('/')));

        FILE *fp = fopen(path.c_str(), "rb");
        fseek(fp, 0, SEEK_END);
        this->size = ftell(fp);
        fclose(fp);
    }
};

class DirAccess
{
private:
    std::__cxx11::string dir_path;

public:
    DirAccess(std::__cxx11::string dir_path)
    {
        this->dir_path = dir_path;
    }

    deque<std::__cxx11::string> *get_files()
    {
        struct dirent **namelist;
        int i, n;
        deque<std::__cxx11::string> *list = new deque<std::__cxx11::string>();

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

class Server_socket
{

    int PORT;

    int listening_socket;
    int connected_client_socket;

    struct sockaddr_in address;
    int address_length;

public:
    Server_socket(int PORT)
    {
        // close_socket();
        create_socket();
        this->PORT = PORT;

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(this->PORT);
        address_length = sizeof(address);

        bind_socket();
        set_listen_set();
        accept_connection();
    }

    void create_socket()
    {
        if ((listening_socket = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
        {
            perror("[ERROR] : Socket failed");
            exit(EXIT_FAILURE);
        }
        cout << "[LOG] : Socket Created Successfully.\n";
    }

    void bind_socket()
    {
        if (bind(listening_socket, (struct sockaddr *)&address, sizeof(address)) < 0)
        {
            perror("[ERROR] : Bind failed");
            exit(EXIT_FAILURE);
        }
        cout << "[LOG] : Bind Successful.\n";
    }

    void set_listen_set()
    {
        if (listen(listening_socket, 3) < 0)
        {
            perror("[ERROR] : Listen");
            exit(EXIT_FAILURE);
        }
        cout << "[LOG] : Socket in Listen State (Max Connection Queue: 3)\n";
    }

    void accept_connection()
    {
        if ((connected_client_socket = accept(listening_socket, (struct sockaddr *)&address, (socklen_t *)&address_length)) < 0)
        {
            perror("[ERROR] : Accept");
            exit(EXIT_FAILURE);
        }
        cout << "[LOG] : Connected to Client.\n";
    }

    // int transmit_file(std::__cxx11::string file->path)
    int transmit_file(FileMD *file)
    {
        char buffer[LENGTH];
        bzero(buffer, LENGTH);
        size_t content_length = 0;
        int fs_block_sz;
        // FileMD *file = new FileMD();

        FILE *file_pointer = fopen(file->path.c_str(), "rb");

        if (file_pointer != NULL)
        {
            // cout << "[LOG] : File is ready to Transmit.\n";
        }
        else
        {
            cout << "[ERROR] : File loading failed, Exiting.\n";
            exit(EXIT_FAILURE);
        }

        bzero(buffer, LENGTH);

        sprintf(buffer, "PING<^v>%ld<^v>%s", file->size, file->name.c_str());
        // Sending Headers = PING+char[128]+ld[32] separated by '<^v>'
        // cout << "[LOG] : Sending Headers...\n";
        if (send(connected_client_socket, buffer, HEADER_LENGTH, 0) <= 0)
        {
            perror("[Error] : Failed to send Headers\n");
            return 0;
        }

        char len[LENGTH];
        strcpy(len, buffer);
        bzero(buffer, LENGTH);

        // cout << "[LOG] : Waiting for Acknowledgement...\n";
        if (read(connected_client_socket, buffer, HEADER_LENGTH) <= 0)
        {
            perror("[Error] : Acknowledgement failed\n");
            return 0;
        }
        // Verifying Sent Headers == Acknowledge Headers
        if (strcmp(len, buffer) != 0)
        {
            printf("[Error] : Integrity Check failed\ntemp: %s|\nbuff: %s|\n", len, buffer);
            return 0;
        }

        // cout << "[LOG] : Acknowledgement Received. Sending FileMD...\n";

        fs_block_sz = fread(buffer, sizeof(char), LENGTH, file_pointer);

        while (fs_block_sz > 0)
        {
            if (fs_block_sz != LENGTH)
            {
                // cout<<(file->size%LENGTH)<<" "<<fs_block_sz<<endl;
            }

            if (content_length += send(connected_client_socket, buffer, fs_block_sz, 0) < 0)
            {
                printf("[ERROR] : Failed to send file_pointer %s.\n", file->path.c_str());
                break;
            }
            bzero(buffer, LENGTH);
            fs_block_sz = fread(buffer, sizeof(char), LENGTH, file_pointer);
        }

        // cout << "[LOG] : Transmitted Data Size " << content_length << " Bytes.\n";

        // cout << "[LOG] : Waiting for Post-Trasfer Acknowledgement...\n";
        if (read(connected_client_socket, buffer, HEADER_LENGTH) <= 0)
        {
            perror("[Error] : Acknowledgement failed\n");
            return 0;
        }
        char temp[LENGTH];
        bzero(temp, LENGTH);
        sprintf(temp, "%ld", file->size);
        // Verifying Sent Headers == Acknowledge Headers
        if (strcmp(temp, buffer) != 0)
        {
            printf("[Error] : Integrity Check failed\ntemp: %s|\nbuff: %s|\n", temp, buffer);
            return 0;
        }

        // printf("[SUCCESS] : Acknowledgement Verified\n[LOG] : File Transfer Complete.\n");
        return 1;
    }

    int transmit_folder_files(std::__cxx11::string dir_path)
    {
        DirAccess *dm = new DirAccess(dir_path);
        deque<std::__cxx11::string> *dir_list = dm->get_files();
        for (int i = 0; i < dir_list->size(); i++)
        {
            // cout << "[" << i << "] " << dir_list->at(i) << endl;
            FileMD *file = new FileMD(dir_path + dir_list->at(i));
            if (transmit_file(file) == 0)
            {
                return 0;
            }
            delete (file);
        }
        delete (dir_list);
    }

    void close_socket()
    {
        close(connected_client_socket);
        close(listening_socket);
    }

    ~Server_socket()
    {
        close_socket();
    }
};
