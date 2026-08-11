#!/bin/bash

printf "%-30s | %-20s | %-10s | %-10s\n" "Repository" "Branch" "Ahead" "Behind"
echo "-------------------------------------------------------------------------------------------"

for dir in */ ; do
    if [ -d "$dir/.git" ]; then
        cd "$dir"

        # Get branch name
        branch=$(git rev-parse --abbrev-ref HEAD)

        # Get upstream branch
        upstream=$(git rev-parse --abbrev-ref --symbolic-full-name @{u} 2>/dev/null)

        if [ -n "$upstream" ]; then
            # Get number of commits ahead and behind
            stats=$(git rev-list --left-right --count $branch...$upstream 2>/dev/null)
            behind=$(echo $stats | awk '{print $1}')
            ahead=$(echo $stats | awk '{print $2}')
        else
            behind="-"
            ahead="-"
        fi

        printf "%-30s | %-20s | %-10s | %-10s\n" "$dir" "$branch" "$behind" "$ahead"

        cd - > /dev/null
    else
        printf "%-30s | %-20s | %-10s | %-10s\n" "$dir" "Not a repo" "-" "-"
    fi
done

echo "-------------------------------------------------------------------------------------------"
echo "Branch status check completed."
