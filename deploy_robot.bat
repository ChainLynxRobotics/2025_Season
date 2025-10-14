@echo on
set JAVA_HOME="C:\Users\Public\wpilib\2024\jdk"
setlocal

git fetch
:deploy_main

git stash
git checkout main

call gradlew deploy
if %errorlevel% neq 0 (
    echo Deployment failed!
    exit /b 1
)
echo Deployment successful!

:end
pause
exit /b
