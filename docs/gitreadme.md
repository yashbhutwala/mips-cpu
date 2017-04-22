How we will Git

We are using git for version control. While git has a standard workflow, it helps to make sure we all know what patterns to follow. 

Workflow. (using a branch called PeterWork to represent a new branch)
(in master branch)

1. make sure your master version is up to date with the remote
    
    git pull 
2. create and switch to new branch

   git branch PeterWork
   git checkout PeterWork

3. Make your changes, TEST YOUR CHANGES, add and commit your changes to the new branch

   git add filename
   git commit -m "this is my descriptive commit statement"

4. switch back to master, MAKE SURE MASTER IS UP TO DATE,  merge your branch into master. 
NOTE: when you type a merge command the syntax it 

git merge [TARGET]
meaning you should be in the master branch and target should be the branch you want merged into master
   
   git checkout master
   git pull
   git merge PeterWork

5. If merge conflicts occur, you have to resolve them. This can be complicated. the best way to avoid these conflicts is branch and merge frequently and to avoid making changes to the same peice of code as someone else at the same time. Unfortunately  Merge conflicts will occur.
   
Hot Tips!

1) to get a quick look at the branch tree on your local machine type in 
   
   git log --oneline --decorate

which returns something like this when your master branch is ready to push

66c6708 (HEAD, peter, master) peter.md added
77652ac (origin/master) readme changes
94c7252 structure
dab87fd setting up structure
0b4973d Testing commit

In this state, the remote is behind the local master and all local branches are up to the same commit. 

2) typing 
   	  git branch
returns a list of all branches with an asterisk next to the branch you have checked out
ex
[pvu001@brki164-lnx-12 docs]$ git branch
* master
  peter
   
