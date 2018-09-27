#!/bin/bash -e

# Quit on errors.
set -o errexit -o nounset

# Settings.
DOCUMENTATION_PATH=documentation
CHANGESET=$(git rev-parse --verify HEAD)

# Set username and email. Hide email from crawlers.
git config user.name "$GH_USER_NAME"
git config user.email "$GH_USER_EMAIL"

# Make documentation.
doxygen Doxyfile

# Commit documentation in master.
cd ..
git add ${DOCUMENTATION_PATH}
git commit -m "[ci skip] Automated documentation build for changeset ${CHANGESET}."
git push -u origin master

echo "-- Successfully updated documentation!"
