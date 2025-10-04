# 🧠 Git Workflow Guide for Beginners
Welcome to our project! To keep our code organized and collaborative, we use a Gitflow workflow. This guide will walk you through the steps for contributing using branches and pull requests (PRs).

## What is Gitflow?

Gitflow is simply a process that allows many people to work in the same project code at the same time while keeping things from getting overwritten, broken, or lost. Instead of directly changing the main code (main branch) 
you submit *pull requests* which are like drafts. Pull requests in our project automatically are scanned to make sure the code compiles which is a first level safeguard to prevent breaking changes from accidentally affecting the main
branch's code. Second, pull requests are reviewed by your peers on the team to make sure the rest of the team is in agreement with the changes and help spot any logic mistakes. This process also helps make sure there aren't conflicts
when two people make changes to the same file.

The following sections will describe the basic process.

## 🚀 Step-by-Step Gitflow Process

### Clone the Repository

```bash
git clone https://github.com/cscsRobotics/ajax-season-2026-decode.git
```

### Create a New Branch
*Always start by creating a new branch from main*

```bash
git checkout main
git pull origin main
git checkout -b your-feature-name
```
🔹 Use short, descriptive names like fix-robot-wheel or add-autonomous-drive.

Make Your Changes Edit files, add new code, and test your changes locally.

### Stage and Commit Your Work

```bash
git add .
git commit -m "Brief description of your changes"
```

### Push Your Branch to GitHub

```bash
git push origin your-feature-name
```

### Open a Pull Request (PR)

1. Go to the GitHub repo.
2. Click Compare & pull request.
3. Add a clear title and description.
4. Submit the PR to merge your branch into main.
5. Wait for Review

Your teacher or teammates will review your code.

You may be asked to make changes—don’t worry, that’s part of learning!

Merge the PR Once approved, your PR will be merged into main.

### ✅ Rules We Follow
- One branch per feature or fix
- No direct commits to main
- Every change must go through a PR
- Write clear commit messages

Ask questions if you're stuck!
