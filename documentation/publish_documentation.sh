#!/bin/bash -e

# Quit on errors.
set -o errexit -o nounset

# Settings.
REPO=`git config remote.origin.url`
SSH_REPO=${REPO/https:\/\/github.com\//git@github.com:}
SHA=`git rev-parse --verify HEAD`
echo "SSH_REPO: ${SSH_REPO}"

# Set username and email. Hide email from crawlers.
git config user.name "$GH_USER_NAME"
git config user.email "$GH_USER_EMAIL"

# Make documentation.
#doxygen Doxyfile

# Commit documentation in master.
#cd ..
git add ${DOCUMENTATION_PATH}
git commit -m "[ci skip] Automated documentation build for changeset ${CHANGESET}."
git push -u $SSH_REPO master

echo "-- Successfully updated documentation!"
