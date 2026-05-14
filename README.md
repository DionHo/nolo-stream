# NoloStream

NoloStream is a simple library written in Rust to stream Pose information from NoloVR to other applications. It uses TCP, UDP or Websockets to send the data, and it is designed to be easy to use and integrate into your projects. The core connection logic is based on https://github.com/lonetech/nolo-osvr (many thanks!).

Project structure:
 - .devcontainer: Development container used for github actions and local development, contains all the dependencies and tools needed to build and test the project.
 - .github: GitHub configuration files, including workflows for continuous integration and deployment.
 - dist: Compiled binaries and distribution files for the library.
 - miniviz: A simple visualization tool to test the streaming of Pose information from NoloVR. It can be used to verify that the library is working correctly and to visualize the data being streamed.
 - src: Source code for the library, including the main NoloStream struct and any helper structs or functions.

## How to use

You can use the binaries in the dist folder to run the streaming server:

```bash
# Start a tcp or websocket server, listening to incoming connections on port 12345
./dist/nolostream_server --tcp-listen-at 12345
./dist/nolostream_server --ws-listen-at 12345

# Or stream directly to a target via tcp or udp
./dist/nolostream_server --tcp-stream-to 192.168.1.100:12345
./dist/nolostream_server --udp-stream-to 192.168.1.100:12345

# Or use any comnbination of the above, for example stream to a target and listen for incoming connections at the same time
./dist/nolostream_server --tcp-listen-at 12345 --tcp-stream-to 192.168.1.100:12345
```

Then, you can use the miniviz tool to connect to the server and visualize the data:

```bash
# Connect to the server via websocket and visualize the data
./dist/miniviz --connect ws://127.0.0.1:12345
```
