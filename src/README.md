Put your ROS2 packages into this folder. This folder is mounted in Docker container.

With the script `git_clone_minimal_setup.sh` you can clone the minimal required repositories for the real robot setup.

With the script `git_fetch_all.sh` you can fetch all updates for the cloned repositories. You still need to pull the changes manually inside the respective folders.

With the script `git_status_all.sh` you can see the current status of all repositories in this folder. You need to run `git_fetch_all.sh` before.