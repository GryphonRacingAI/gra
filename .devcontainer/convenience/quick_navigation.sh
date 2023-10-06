depends_var RBC_REPO
depends_var CATKIN_WS_PATH

function cdrepo() {
    cd $RBC_REPO
}

function cdr() {
    cdrepo
}

function cdws() {
    cd $CATKIN_WS_PATH
}

function cdw() {
    cdws
}

export -f cdrepo
export -f cdr
export -f cdws
export -f cdw