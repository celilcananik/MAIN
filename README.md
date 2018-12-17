# GITHUB USAGE FOR DUMMIES

# Updating Local Folder ( Pulling Project to Local )
    git pull origin master
    git pull origin my_default_branch_name

# Deleting Dummy Folders
The steps for doing this are:

    - In the command-line, navigate to your local repository.
    - Ensure you are in the default branch:
      git checkout master
    - The rm -r command will recursively remove your folder:
      git rm -r folder-name
    - Commit the change:
      git commit -m "Remove duplicated directory"
    - Push the change to your remote repository:
      git push origin master
