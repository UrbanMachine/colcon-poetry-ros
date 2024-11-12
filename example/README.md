# Example

This is an example project that uses colcon-poetry-ros. We use Docker to
illustrate setup, but the process is the same outside a container.

## Running

Run the following commands _while in the project root_.

```bash
docker build --tag colcon-poetry-ros-example --file example/Dockerfile .
docker run -it colcon-poetry-ros-example
```
