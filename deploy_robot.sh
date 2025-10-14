#!/bin/bash

MAIN_BRANCH="main"
DEPLOY_COMMAND="./gradlew deploy"

PROJECT_DIR=$(pwd)
cd $PROJECT_DIR || { echo "Failed to navigate to the project directory"; exit 1; }

echo "Stashing local changes..."
git stash

echo "Switching to the $MAIN_BRANCH branch..."
git checkout $MAIN_BRANCH

echo "Pulling the latest changes from $MAIN_BRANCH..."
git pull origin $MAIN_BRANCH

echo "Deploying robot code to the robot..."
$DEPLOY_COMMAND

if [ $? -eq 0 ]; then
    echo "Deployment successful!"
else
    echo "Deployment failed. Check the logs for details."
    exit 1
fi

echo "Applying stashed changes..."
git stash pop
