# autodrive-iros2024-qualification

+ This code was used for autodrive iros2024 qualification

``` bash
# 1. git clone
git clone https://github.com/IdeaLabGNU-F1TENTH/autodrive-iros2024-qualification.git

# 2. change directory
cd autodrive-iros2024-qualification

# 3. build docker
docker build -t autodrive_idea_lab:v1 .

# 4. Enable display forwarding
xhost local:root

# 5. run docker
docker run --name autodrive_f1tenth_api --rm -it --entrypoint /bin/bash --network=host --ipc=host -v /tmp/.X11-unix:/tmp.X11-umix:rw --env DISPLAY --privileged --gpus all autodrive_idea_lab:v1

# 6. launch code
sh autodrive.sh
```
