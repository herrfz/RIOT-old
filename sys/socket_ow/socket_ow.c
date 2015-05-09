#include <errno.h>

#include "socket_base/socket.h"
#include "fd.h"

#include "sys/socket.h"

int flagless_send(int fd, const void *buf, size_t len)
{
    return (int)socket_base_send(fd, buf, (uint32_t)len, 0);
}

int flagless_recv(int fd, void *buf, size_t len)
{
    return (int)socket_base_recv(fd, buf, (uint32_t)len, 0);
}

int socket(int domain, int type, int protocol)
{
    int internal_socket = socket_base_socket(domain, type, protocol);

    if (internal_socket < 0) {
        errno = ENFILE;
        return -1;
    }

    return fd_new(internal_socket, flagless_recv, flagless_send,
                  socket_base_close);
}


#define sock_func_wrapper(func, sockfd, ...) \
    ((fd_get(sockfd)) ? \
        func(fd_get(sockfd)->internal_fd, __VA_ARGS__) : \
        (errno = EBADF, -1))


int bind(int socket, const struct sockaddr *address, socklen_t address_len)
{
    int res = sock_func_wrapper(socket_base_bind, socket,
                                (sockaddr6_t *)address, address_len);

    if (res < 0) {
        // transport_layer needs more granular error handling
        errno = EOPNOTSUPP;
        return -1;
    }

    return res;
}

// TODO sendto
/*
ssize_t sendto(int socket, const void *message, size_t length, int flags,
               const struct sockaddr *dest_addr, socklen_t dest_len)
{
    int32_t res = sock_func_wrapper(socket_base_sendto, socket, message, // TODO change wrapped func with openudp_send?
                                    (uint32_t) length, flags,
                                    (sockaddr6_t *)dest_addr,
                                    (socklen_t)dest_len);

    if (res < 0) {
        // transport_layer needs more granular error handling
        errno = ENOTCONN;
        return -1;
    }

    return (ssize_t)res;
}
*/

ssize_t recvfrom(int socket, void *restrict buffer, size_t length, int flags,
                 struct sockaddr *restrict address,
                 socklen_t *restrict address_len)
{
    int32_t res = sock_func_wrapper(socket_base_recvfrom, socket, buffer,
                                    (uint32_t) length, flags,
                                    (sockaddr6_t *)address,
                                    (socklen_t *)address_len);

    if (res < 0) {
        // transport_layer needs more granular error handling
        errno = ENOTCONN;
        return -1;
    }

    return (ssize_t)res;
}