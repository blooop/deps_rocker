#! /bin/bash

#THIS IS THE CORRECT TEMPLATE UPDATE SCRIPT, NOT update_from_python_template.sh

git config pull.rebase false
git remote add template https://github.com/blooop/template_rocker.git
git fetch --all
git checkout main && git pull origin main
git checkout -B feature/update_from_template; git pull
git merge template/main --allow-unrelated-histories -m 'feat: pull changes from remote template'
git checkout --ours pixi.lock; git add pixi.lock
git remote remove template
git push --set-upstream origin feature/update_from_template
git checkout main
