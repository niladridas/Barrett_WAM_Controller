# rename origin remote
git add *
echo ADDED
git commit -m "$1"
echo committed
git push -u github master
git push -u bitbucket master