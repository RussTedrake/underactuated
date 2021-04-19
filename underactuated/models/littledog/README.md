Russ can access the original files in two repos:
`/afs/csail.mit.edu/u/r/russt/locomotion_svn/robots/LittleDog/`
(e.g.the original CAD files are in `trunk/3ds`)

And the main littledog repo can be checked out with:
`svn co file:///afs/csail.mit.edu/u/r/russt/littledog/.svnroot littledog_svn --depth immediates`
and then inspected with selective use of e.g.
`svn list` and 
`svn update --set-depth immediates`

