#!/bin/bash

# Loop through all directories in the current folder
for dir in */ ; do
    # Check if the directory is a git repository
    if [ -d "$dir/.git" ]; then
        echo -e "\nFetching updates in $dir"
        (cd "$dir" && git fetch --all)
    else
        echo "$dir is not a Git repository. Skipping."
    fi
done

echo -e "\nFetch operation completed."
