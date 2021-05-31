# Learn how to git

https://www.youtube.com/watch?v=Y9XZQO1n_7c \
git uses a local file base set up\
need to backup locally or commit to a remote repository

## git start
```bash
git init # .git folder where git stores everything including changes sets, branches
git status # tells what branch you are in, untracked files, changes not staged for commit
```

## git add/commit
```bash
git add # tells git to stage file, when you add you save the version of the file at that time
git add . # :tells git to stage everything
# * use .gitignore file to exclude files or file types ie *.log
git add *.html # tells git to stage only files with html
git commit # vim popup for comment message. i: insert, esc: exit insert mode, :w save, :q quit vim
git commit -m # saves comment message without vim
git commit -a # any changes to files that are currently tracked will be automatically staged, new files wont be staged and need to be manually added
git commit -a -m 'my message' # commit all and save comment without vim
```

## git remove
```bash
git rm -r (filename.txt) # remove a file or folder
git rm --cached (filename) # remove a file from staged
git rm --cached -r # reset all repo
```

## git commit history
```bash
git log # commit history, list out commits and their details like author, date, and comment messages
git log --oneline # shortened commit history
git diff (source) (targetbranch) # preview changes before merging
```

## git branch
```bash
git branch # list branches
git branch (branchname) # creates new branch with name
git branch -d (branch name) # delete a branch
git checkout (branchname) # switch to branchname, or switch to master branch
git checkout -b (branchname) # creates branch if does not exist
```

## git stash
```bash
git stash # take dirty state of branch including tracked modifications and staged changes and saves it on a stack of unfinished changes that can be reapplied at any time
# when we add and commit then switch to different branch, files disappear/appears
# when we makes changes but dont commit them, the changes remain in the branches
git stash apply # reapply changes from the stash to branch
git stash clear # remove all stashed entries
```

## git remote
```bash
git remote # list remote repositories
git remote -v # list fetch and push remote repos with alias origin
git remote add (aliasname) (url) # add additional remote repositories
git remote set-url (aliasname) (url) #  set repo alias to new url
git remote rename (newname) (oldname) # rename remote repo
git clone https://github.com/name/repo.git # copy remote repository to local folder, inclcudes commit history
```

## git fetch/pull/push
```bash
git fetch origin # go to server and get any changes made since last clones or fetched
# need to manually merge it into your work
git fetch (alias name) # copy from remote repo to choosen repo
git pull origin # automatically fetch and merge the changes from the remote branch into current branch
# before pushing need to commit
git push origin master # save local changes to the remote repository known as origin in master branch
# need to login credentials
```

## git merge
```bash
# before merge need to make sure that you are on the destination branch
git merge (source branch) # merge source branch to destination
# conflicts: head is the version in the current branch, master is the version in the source branch
git merge (source) (target branch) # merge a branch into target branch
git mergetool # need external program to help merge conflicts ie tortisemerge, winmerge
git merge --ff-only # fast forward merge
```

## git rebase
```bash
git rebase # if merge --ff-only failed, rebase commit puts current commit at end of history commit 
```

## git Go back previous versions
```bash
git checkout -b my_release (commit-id)
# ...prepare code for release...
# ...release code...
git checkout master
git merge my_release
```

## git quick steps
1. git init (use git status)
2. git remote add origin (url)
3. git add .
4. git commit -a -m 'my message'
5. git push origin master

## git at work
1. git remote update
2. git merge --ff-only
3. git commit -a -m 'my message'
4. git push gitlab branch
