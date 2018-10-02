#!/bin/bash

# Quit on errors.
set -e

# Settings.
REPO=`git config remote.origin.url`
SSH_REPO=${REPO/https:\/\/github.com\//git@github.com:}
SHA=`git rev-parse --verify HEAD`
echo "SSH_REPO: ${SSH_REPO}"

# Set username and email. Hide email from crawlers.
git config user.name "$GH_USER_NAME"
git config user.email "$GH_USER_EMAIL"

# Commit documentation in master.
git add --all documentation/
git commit -am "[ci skip] Automated documentation build for changeset ${SHA}."
git push $SSH_REPO master

echo "-- Successfully updated documentation!"
