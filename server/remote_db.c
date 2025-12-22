#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <signal.h>
#include <mysql/mysql.h>

#define BUF_SIZE 400
#define NAME_SIZE 20
#define ARR_CNT 5

void* send_msg(void* arg);
void* recv_msg(void* arg);
void error_handling(char* msg);

char name[NAME_SIZE] = "[Default]";
char msg[BUF_SIZE];

int main(int argc, char* argv[])
{
	int sock;
	struct sockaddr_in serv_addr;
	pthread_t snd_thread, rcv_thread;
	void* thread_return;

	if (argc != 4) {
		printf("Usage : %s <IP> <port> <name>\n", argv[0]);
		exit(1);
	}

	sprintf(name, "%s", argv[3]);

	sock = socket(PF_INET, SOCK_STREAM, 0);
	if (sock == -1)
		error_handling("socket() error");

	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
	serv_addr.sin_port = htons(atoi(argv[2]));

	if (connect(sock, (struct sockaddr*) & serv_addr, sizeof(serv_addr)) == -1)
		error_handling("connect() error");

	sprintf(msg, "[%s:PASSWD]", name);
	write(sock, msg, strlen(msg));
	pthread_create(&rcv_thread, NULL, recv_msg, (void*)&sock);
	pthread_create(&snd_thread, NULL, send_msg, (void*)&sock);

	pthread_join(snd_thread, &thread_return);
	pthread_join(rcv_thread, &thread_return);

	if(sock != -1)
		close(sock);
	return 0;
}


void* send_msg(void* arg)
{
	int* sock = (int*)arg;
	int str_len;
	int ret;
	fd_set initset, newset;
	struct timeval tv;
	char name_msg[NAME_SIZE + BUF_SIZE + 2];

	FD_ZERO(&initset);
	FD_SET(STDIN_FILENO, &initset);

	fputs("Input a message! [ID]msg (Default ID:ALLMSG)\n", stdout);
	while (1) {
		memset(msg, 0, sizeof(msg));
		name_msg[0] = '\0';
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		newset = initset;
		ret = select(STDIN_FILENO + 1, &newset, NULL, NULL, &tv);
		if (FD_ISSET(STDIN_FILENO, &newset))
		{
			fgets(msg, BUF_SIZE, stdin);
			if (!strncmp(msg, "quit\n", 5)) {
				*sock = -1;
				return NULL;
			}
			else if (msg[0] != '[')
			{
				strcat(name_msg, "[ALLMSG]");
				strcat(name_msg, msg);
			}
			else
				strcpy(name_msg, msg);
			if (write(*sock, name_msg, strlen(name_msg)) <= 0)
			{
				*sock = -1;
				return NULL;
			}
		}
		if (ret == 0)
		{
			if (*sock == -1)
				return NULL;
		}
	}
}

void* recv_msg(void* arg)
{
	MYSQL* conn;
	MYSQL_ROW sqlrow;
	int res;
	char sql_cmd[BUF_SIZE] = { 0 };
	char* host = "localhost";
	char* user = "iot";
	char* pass = "asdf;lkj";
	char* dbname = "iotdb";

	int* sock = (int*)arg;
	int i;
	char* pToken;
	char* pArray[ARR_CNT] = { 0 };

	char name_msg[NAME_SIZE + BUF_SIZE + 1];
	int str_len;

	int illu;
	float temp;
	float humi;

    char* signal_name;
    char* uploader;
    char* signal; 
	conn = mysql_init(NULL);

	puts("MYSQL startup");
	if (!(mysql_real_connect(conn, host, user, pass, dbname, 0, NULL, 0)))
	{
		fprintf(stderr, "ERROR : %s[%d]\n", mysql_error(conn), mysql_errno(conn));
		exit(1);
	}
	else
		printf("Connection Successful!\n\n");

	while (1) {
		memset(name_msg, 0x0, sizeof(name_msg));
		str_len = read(*sock, name_msg, NAME_SIZE + BUF_SIZE);
		if (str_len <= 0)
		{
			*sock = -1;
			return NULL;
		}
		fputs(name_msg, stdout);
//		name_msg[str_len-1] = 0;   //'\n' 제거
		name_msg[strcspn(name_msg,"\n")] = '\0';

		pToken = strtok(name_msg, "[:@]");
		i = 0;
		while (pToken != NULL)
		{
			pArray[i] = pToken;
			if ( ++i >= ARR_CNT)
				break;
			pToken = strtok(NULL, "[:@]");

		}
		if(!strcmp(pArray[1],"IRSAVE") && (i == 3)){
			signal = pArray[2];
			char escaped_signal[BUF_SIZE];  
			mysql_real_escape_string(conn, escaped_signal, pArray[2], strlen(pArray[2]));

  			sprintf(sql_cmd,
                "INSERT INTO remote(signal_name, ir_signal, uploader, create_time)"
                "VALUES ('%s', '%s', '%s', NOW());", "Unknown IR", signal, "이동현");
			
				res = mysql_query(conn, sql_cmd);
			if (!res) {
				unsigned long last_id = (unsigned long)mysql_insert_id(conn);
				printf("inserted %lu rows, last_id=%lu\n",
					(unsigned long)mysql_affected_rows(conn), last_id);

				char response[128];
				// [LDH_SMP]IRSAVE@{id}\n
				int n = snprintf(response, sizeof(response), "[LDH_SMP]IRSAVE@%lu\n", last_id);
				if (n > 0) write(*sock, response, (size_t)n);
			} else {
				fprintf(stderr, "ERROR: %s[%d]\n", mysql_error(conn), mysql_errno(conn));
			}
		}

		else if (!strcmp(pArray[1], "GETIR") && i >= 3)
		{
			long id = strtol(pArray[2], NULL, 10);

			snprintf(sql_cmd, sizeof(sql_cmd),
					"SELECT ir_signal FROM remote WHERE id=%ld LIMIT 1", id);
			mysql_query(conn, sql_cmd);

			MYSQL_RES *result = mysql_store_result(conn);
			MYSQL_ROW row = mysql_fetch_row(result);

			if (row && row[0]) {
				snprintf(sql_cmd, sizeof(sql_cmd),
						"[%s]SHOT@%s\n", "LDH_ARD", row[0]);
			} 

			write(*sock, sql_cmd, strlen(sql_cmd));
			if (result) mysql_free_result(result);
		}
	}
	mysql_close(conn);
}

void error_handling(char* msg)
{
	fputs(msg, stderr);
	fputc('\n', stderr);
	exit(1);
}
