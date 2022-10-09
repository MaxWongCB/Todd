# Todd
## Notes
<b>Add any important notes about running files/setup commands here.</b>

### Model Training
```model``` folder from ```Week06-07/network/scripts``` has been removed as they are too big to upload. 
I have zipped it up and put them on the slack channel so if you need them you can access from there then add them to your local copy.

### Rebuilding the catkin_ws
Has not been included here as there was complications with building the workspace then loading to Github.There is a copy of the latest ```catkin_ws``` on the slack channel if you need it. You will likely have to rebuild the catkin workspace if you do so(instructions can be found in install guide on main ECE4078 Lab repo).

## Git commands
<b>Add git commands for commits/pushes etc here...</b>
### Setting up user credentials
To set Github user and email in WSL:<br></br>
```git config --global user.name <YourGithubUserName>```<br></br>
```git config --global user.email <Your GithubEmail>```

### Creating Branches
After cloning the Repo, you can create a branch from the master with: <br></br>
```git branch <NewBranchName>```<br></br>
To then switch to the new branch use:<br></br> 
```git checkout <NewBranchName>```<br></br>

To view branches on your local wsl use ```git branch``` or to view all repos on local and the remote Github Repo use ```git branch -a```<br></br>
I'd recommend only using ```git checkout``` to switch between branches on your local WSL when working on the code.

### Making commits/Pushes
When you want to save some changes to the code, save the file(s) you're working on as normal. 
1. First make sure you're working in the correct branch with ```git branch```. It should highlight with an asterisk which branch you are working from. Use ```git checkout <WorkingBranch>``` to switch to the one you want.
2. To add modified files to the staging area use ```git add .```
3. To then add these modified files to the next commit use ```git commit -m "Commit Message"```
4. To push to the remote branch use ```git push``` Note: the first time you set up a branch it will ask for a slightly modified command ```git push --set-upstream origin <NewBranchName>```<br></br> From then on ```git push``` should be all you need.
### Merging with master branch
...
### other useful commands
...
