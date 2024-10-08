#! /bin/bash

#THIS IS USED TO UPDATE THIS TEMPLATE FROM THE MAIN PYTHON TEMPLATE https://github.com/blooop/python_template.  YOU SHOULD NOT NEED TO USE THIS, PLEASE USE update_from_template.sh INSTEAD IF YOU WANT TO PULL UPDATES FROM TEMPLATE_ROCKER

git config --global pull.rebase false
git remote add template https://github.com/blooop/python_template.git
git fetch --all
git checkout main && git pull origin main
git checkout -B feature/update_from_template; git pull
git merge template/main --allow-unrelated-histories -m 'feat: pull changes from remote template'
git checkout --ours pixi.lock; git add pixi.lock
git remote remove template
git push --set-upstream origin feature/update_from_template
git checkout main
