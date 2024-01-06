## Label template directory 

Style guide: https://docs.google.com/document/d/1WuY5wWGJLnJvZWReWLSVGsFmB0r_jXAS9LVLu90_P8g/edit?usp=sharing

Start working on a new repo
- Clone 
- Pre-commit install
- Update xyz
- First pull request merged, add name to contributors

Start working on a new branch with your feature
In Terminal/Command Prompt,

        git fetch
        git checkout origin/main
        git checkout -b <YourName>/<branchname>

- Fetch latest main
- Checkout branch you want to work off of–usually main
- Checkout your new branch

        git status
        git add path/To/File // looks like that or the + button in the vs code interface
        // if you want to add everything, use git add * or git add src/
        git commit -m “your commit message”
        git push

- Check what files have changed and double check your branch. If a strange file has been changed that you have not touched, make sure to check the difference using  ‘git diff’.
- Add code to commit
- Commit code
- Push branch--if it is a new branch, the git push will fail and there will be a branch specific command to push the new branch to github


This directory contains templates used in the `add-template-to-issue.yml` workflow

Each template must have a filename of `label-<name of label>.md` to automatically be applied to a label of that name

For example, if you would like to add a template to an issue when the `badbug` label is applied, add a file in this directory called

`label-badbug.md`

The template will be added as a new comment as to avoid muddling existing comments
