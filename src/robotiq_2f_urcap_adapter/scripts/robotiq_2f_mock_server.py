#!/usr/bin/env python3

# This script is a mock server for the Robotiq 2F gripper. It listens for incoming connections from a client and
# processes the commands sent by the client. The server responds to the client with the requested data or an
# acknowledgment message. It also simulates the gripper's behavior by updating the register values based on the
# received commands.

import socket

import rclpy
import sys


def start_server(host="localhost", port=63352):
    # Create a socket object
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind to the server address and port
    server_socket.bind((host, port))

    # Start listening for connections
    server_socket.listen(5)
    print(f"Server started and listening on {host}:{port}")

    # Dictionary to hold register values
    registers = {}

    try:
        while True:
            # Accept a client connection
            client_socket, client_address = server_socket.accept()
            print(f"Connection from {client_address} has been established.")

            while True:

                if registers.get("ACT", "0") == "1":
                    registers["STA"] = "3"
                if registers.get("GTO", "0") == "1":
                    registers["OBJ"] = "3"

                # Receive data from the client
                data = client_socket.recv(1024).decode("utf-8")
                if not data:
                    break  # No more data from client, break the loop

                # print(f"Received data: {data}")

                parts = data.split()
                if len(parts) < 2:
                    response = "Invalid command"
                else:
                    command = parts[0]
                    register = parts[1]

                    if command == "GET":
                        value = registers.get(
                            register, "0"
                        )  # Return '0' if register is not found
                        response = f"{register} {value}"
                    elif command == "SET":
                        if len(parts) % 2 == 0:
                            response = "Invalid"
                        else:
                            response = []
                            for i in range(1, len(parts), 2):
                                register = parts[i]
                                value = parts[i + 1]
                                registers[register] = value
                                if register == "POS":
                                    registers["PRE"] = value
                            response = "ack"
                    else:
                        response = "Invalid command"

                # print(f"Sending response: {response}")
                # Send response to the client
                client_socket.sendall(response.encode("utf-8"))

            # Close the client connection after the client has finished sending data
            client_socket.close()
            print(f"Connection with {client_address} closed.")

    except KeyboardInterrupt:
        print("Server is shutting down.")
    finally:
        # Close the server socket
        server_socket.close()


if __name__ == "__main__":
    port = 63352
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
    rclpy.init(args=None)
    node = rclpy.create_node("robotiq_2f_mock_server")
    start_server(port=port)
