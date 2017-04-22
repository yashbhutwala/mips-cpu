Git Test!

following the instructions complete the following

1) make a branch and check it out
2) on that branch, make a file yashstinks.md that says something like "I am a branch boy"
3) add and commit your new file while still on your branch
4) switch to master branch, ls, notice that your yashstinks.md is gone
5) update the master branch with a git pull
6) merge your new branch into your master branch 
7) make a note of the 
git log --oneline --decorate

8) normally this is where we would push our changes to the remote but since we don't want yashstinks.md we will remove that file and delete our branch

   rm yashstinks.md
   git branch -d [BRANCHNAME]
