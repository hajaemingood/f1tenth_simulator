cd ~/f1tenth_simulator/docker_f1tenth
xhost +local:docker
docker compose up -d --build
docker exec -it ros2_foxy bash
