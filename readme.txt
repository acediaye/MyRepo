learn how to git

https://www.youtube.com/watch?v=Y9XZQO1n_7c
git uses a local file base set up
need to backup manually or commit to a remote repository

git init: .git where git stores everything including changes sets, branches
git status: tells what branch you are in, untracked files, changes not staged for commit

git add: tells git to stage file, when you add you save the version of the file at that time
git add .   tells git to stage everything
			use .gitignore file to exclude files or file types ie *.log
git add *.html     tells git to stage only files with html
git commit: vim popup for comment message. i: insert, esc: exit insert mode, :w save, :q quit vim
git commit -m: saves comment message without vim
git commit -a: any changes to files that are currently tracked will be automatically staged, new files wont be staged and need to be manually added

git rm -r <filename.txt>: remove a file or folder
git rm --cached <filename>: remove a file from staged

git log: commit history, list out commits and their details like author, date, and comment messages
git diff <source> <targetbranch>: preview changes before merging

git branch: list branches
git branch <branchname>: creates new branch with name
git branch -d <branch name>: delete a branch
git checkout <branchname>: switch to branchname, or switch to master branch

	before merge need to make sure that you are on the destination branch
git merge <source branch>: merge source branch to destination
	conflicts: head is the version in the current branch, master is the version in the source branch
git merge <source> <target branch>: merge a branch into target branch
git mergetool: need external program to help merge conflicts ie tortisemerge, winmerge

git stash: take dirty state of branch including tracked modifications and staged changes and saves it on a stack of unfinished changes that can be reapplied at any time
	when we add and commit then switch to different branch, files disappear/appears
	when we makes changes but dont commit them, the changes remain in the branches
git stash apply: reapply changes from the stash to branch
git stash clear: remove all stashed entries

git remote: list remote repositories
git remote -v: list fetch and push remote repos with alias origin
git remote add <aliasname> <url>: add additional remote repositories
git remote set-url <aliasname> <url>:  set repo alias to new url
git clone <https://github.com/<name>/<repo>.git: copy remote repository to local folder, inclcudes commit history

git fetch origin: go to server and get any changes made since last clones or fetched
	need to manually merge it into your work
git fetch <alias name>: copy from remote repo to local repo
git pull origin: automatically fetch and merge the changes from the remote branch into current branch
	before pushing need to commit
git push origin master: save local changes to the remote repository known as origin in master branch
	need to login credentials
