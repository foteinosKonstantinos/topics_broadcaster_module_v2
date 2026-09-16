Build:

```bash
./build.bash
```

Run transmitter (server):

```bash
./run_transmitter.bash
```

Run receiver (client):

```bash
./run_receiver.bash
```

Run producer (dummy data generator):

```bash
./run_producer.bash
```

To change the compression ratio:

```bash
ros2 param list
ros2 param set /receiver_client rgb_compression_quality <quality in 0-100>
ros2 param set /receiver_client depth_compression_quality <quality in 0-100>
```