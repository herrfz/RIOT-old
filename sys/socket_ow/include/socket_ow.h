/*
 * Copyright (C) 2015 TriaGnoSys GmbH
 *
 */

/**
 * @defgroup    sys_socket_ow POSIX socket for OpenWSN
 * @ingroup     sys
 */

/**
 * @file    socket_ow.h
 * @brief   POSIX socket header for OpenWSN
 *
 * @author  Eriza Fazli <eriza.fazli@triagnosys.com>
 */
#ifndef _SYS_SOCKET_OW_H
#define _SYS_SOCKET_OW_H

#ifdef CPU_NATIVE
/* Ignore Linux definitions in native */
#define _BITS_SOCKADDR_H    1
#define __SOCKADDR_COMMON(sa_prefix) \
  sa_family_t sa_prefix##family

#define __SOCKADDR_COMMON_SIZE  (sizeof (unsigned short int))
#endif

#include <sys/types.h>

#include "cpu.h"

#include "socket_base/socket.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Used to define the socket address.
 */
struct __attribute__((packed)) sockaddr {
    sa_family_t sa_family;  ///< Address family
    char sa_data[14];       ///< Socket address (variable length data)
};

/**
 * @brief   Implementation based socket address table.
 */
struct __attribute__((packed)) sockaddr_storage {
    sa_family_t ss_family;  ///< Address family
    char ss_data[14];       ///< address data
};


/*
 * Many are omitted from original specification, consider only connectionless socket
 */

 /**
 * @brief   Create an endpoint for communication.
 * @details Shall create an unbound socket in a communications domain, and
 *          return a file descriptor that can be used in later function calls
 *          that operate on sockets.
 *
 * @param[in] domain    Specifies the communications domain in which a socket
 *                      is to be created. Valid values are prefixed with ``AF_`
 *                      and defined in @ref socket.h.
 * @param[in] type      Specifies the type of socket to be created. Valued
 *                      values are prefixed with ``SOCK_`` and defined in
 *                      @ref socket.h.
 * @param[in] protocol  Specifies a particular protocol to be used with the
 *                      socket. Specifying a protocol of 0 causes socket() to
 *                      use an unspecified default protocol appropriate for
 *                      the requested socket type.
 *
 * @return  Upon successful completion, socket() shall return a non-negative
 *          integer, the socket file descriptor. Otherwise, a value of -1 shall
 *          be returned and errno set to indicate the error.
 */
int socket(int domain, int type, int protocol);


/**
 * @brief   Bind a name to a socket.
 * @details The bind() function shall assign a local socket address *address*
 *          to a socket identified by descriptor socket that has no local
 *          socket address assigned. Sockets created with the socket() function
 *          are initially unnamed; they are identified only by their address
 *          family.
 *
 * @see <a href="http://pubs.opengroup.org/onlinepubs/9699919799/functions/bind.html">
 *          The Open Group Base Specification Issue 7, bind
 *      </a>
 *
 * @param socket        Specifies the file descriptor of the socket to be bound.
 * @param address       Points to a sockaddr structure containing the address
 *                      to be bound to the socket. The length and format of the
 *                      address depend on the address family of the socket.
 *                      If the address family of the socket is AF_UNIX and the
 *                      pathname in address names a symbolic link, bind() shall
 *                      fail and set errno to [EADDRINUSE].
 * @param address_len   Specifies the length of the sockaddr structure pointed
 *                      to by the *address* argument.
 * @return  Upon successful completion, bind() shall return 0; otherwise, -1
 *          shall be returned and errno set to indicate the error.
 */
int bind(int socket, const struct sockaddr *address, socklen_t address_len);


/**
 * @brief   Send a message on a socket.
 * @details Shall send a message through a connection-mode or
 *          connectionless-mode socket. If the socket is a connectionless-mode
 *          socket, the message shall be sent to the address specified by
 *          *dest_addr* if no pre-specified peer address has been set. If a
 *          peer address has been pre-specified, either the message shall be
 *          sent to the address specified by *dest_addr* (overriding the
 *          pre-specified peer address), or the function shall return -1 and
 *          set errno to EISCONN.
 *
 * @see <a href="http://pubs.opengroup.org/onlinepubs/9699919799/functions/sendto.html">
 *          The Open Group Base Specification Issue 7, sendto
 *      </a>
 *
 * @param[in] socket    Specifies the socket file descriptor.
 * @param[in] message   Points to the buffer containing the message to send.
 * @param[in] length    Specifies the length of the message in bytes.
 * @param[in] flags     Specifies the type of message reception. Support
 *                      for values other than 0 is not implemented yet.
 * @param[in] dest_addr Points to a sockaddr structure containing the
 *                      destination address. The length and format of the
 *                      address depend on the address family of the socket.
 * @param[in] dest_len  Specifies the length of the sockaddr structure pointed
 *                      to by the *dest_addr* argument.
 *
 * @return  Upon successful completion, send() shall return the number of bytes
 *          sent. Otherwise, -1 shall be returned and errno set to indicate the
 *          error.
 */
ssize_t sendto(int socket, const void *message, size_t length, int flags,
               const struct sockaddr *dest_addr, socklen_t dest_len);


/**
 * @brief   Receive a message from a socket.
 * @details The recvfrom() function shall receive a message from a
 *          connection-mode or connectionless-mode socket. It is normally used
 *          with connectionless-mode sockets because it permits the application
 *          to retrieve the source address of received data.
 *
 * @see <a href="http://pubs.opengroup.org/onlinepubs/9699919799/functions/recvfrom.html">
 *          The Open Group Base Specification Issue 7, recvfrom
 *      </a>
 *
 * @param[in] socket        Specifies the socket file descriptor.
 * @param[out] buffer       Points to a buffer where the message should be i
 *                          stored.
 * @param[in] length        Specifies the length in bytes of the buffer pointed
 *                          to by the buffer argument.
 * @param[in] flags         Specifies the type of message reception. Support
 *                          for values other than 0 is not implemented yet.
 * @param[out] address      A null pointer, or points to a sockaddr structure
 *                          in which the sending address is to be stored. The
 *                          length and format of the address depend on the
 *                          address family of the socket.
 * @param[out] address_len  Either a null pointer, if address is a null pointer,
 *                          or a pointer to a socklen_t object which on input
 *                          specifies the length of the supplied sockaddr
 *                          structure, and on output specifies the length of
 *                          the stored address.
 *
 * @return  Upon successful completion, recvfrom() shall return the length of
 *          the message in bytes. If no messages are available to be received
 *          and the peer has performed an orderly shutdown, recvfrom() shall
 *          return 0. Otherwise, the function shall return -1 and set errno to
 *          indicate the error.
 */
ssize_t recvfrom(int socket, void *__restrict buffer, size_t length, int flags,
                 struct sockaddr *__restrict address,
                 socklen_t *__restrict address_len);


#ifdef __cplusplus
}
#endif

/**
 * @}
 */
#endif /* SYS_SOCKET_OW_H */
