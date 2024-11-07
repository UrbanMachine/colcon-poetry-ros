ARG ROS_VERSION=humble
FROM osrf/ros:${ROS_VERSION}-desktop

ARG ROS_VERSION=humble
ARG POETRY_VERSION=1.8.3
ARG PIP_VERSION=24.3.1

RUN apt-get update && apt-get install --yes python3-pip pipx python-is-python3
RUN curl -fsSL https://install.python-poetry.org --output /tmp/install-poetry.py \
    && POETRY_HOME=/usr/local python3 /tmp/install-poetry.py --version "${POETRY_VERSION}"
RUN poetry self add poetry-plugin-bundle

RUN pip3 install --upgrade pip==${PIP_VERSION}

# Set up rosdep
RUN rosdep update --rosdistro ${ROS_VERSION}

# Install colcon-poetry-ros
WORKDIR /colcon-poetry-ros
COPY setup.cfg .
COPY setup.py .
COPY README.md .
COPY colcon_poetry_ros colcon_poetry_ros
RUN python3 -m pip install .

WORKDIR /example

# Install example rosdep dependencies
COPY example/src/my_package/package.xml src/my_package/package.xml
RUN rosdep install -i --from-path src --rosdistro "${ROS_VERSION}" -y

# Install Poetry dependencies
COPY example/src/my_package/pyproject.toml src/my_package/pyproject.toml
COPY example/src/my_package/poetry.lock src/my_package/poetry.lock
RUN python3 -m colcon_poetry_ros.dependencies.install --base-paths src

COPY example/src src
RUN colcon build

CMD ["bash", "-c", "source install/setup.bash && ros2 run my_package cowsayer"]
