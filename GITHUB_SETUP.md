# Pushing this repo to GitHub

This folder is already a git repository with an initial commit.

1. Create a new **empty** repo on GitHub (no README/license), e.g.
   `spi-master-cocotb`. Copy its URL.

2. In this folder:

   ```bash
   git remote add origin https://github.com/<your-username>/spi-master-cocotb.git
   git branch -M main
   git push -u origin main
   ```

If you downloaded this as a zip, unzip it first, then run the commands above
from inside the unzipped folder. If `git log` shows the initial commit, you're
good to push.

Prefer the GitHub web uploader? Create the repo, click **Add file -> Upload
files**, and drag in everything except the `.git` folder.
