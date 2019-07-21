# My custom .zshrc aliases and functions
#
#
##############################zsh plugins############################
plugins=(
    git
    zsh-autosuggestions
    zsh-syntax-highlighting
    autojump
    vscode
    ubuntu
    themes
    sudo
    command-not-found
    dircycle
    colorize
    last-working-dir
    web-search
    copydir
    copyfile
    extract
    colored-man-pages
)
#### zsh
ZSH_THEME="powerline-custom"
source $ZSH/oh-my-zsh.sh
setopt nonomatch
export EDITOR='gedit'

############################variables################################
theme_path="/home/eric/.oh-my-zsh/custom/themes"
zsh_git_path="/home/eric/.zsh/zsh_powerline_config"
workspace_ros_path="/home/eric/work_space/workspace_ros"
current_ws="current_ws"
timeonstart=$(date +%y.%m.%d-%H:%M:%S)
backuptime=$(echo $timeonstart | sed 's|^|#### Last backup time: |')
split='-------------------------------------------------------'

##############################aliases################################
#### zsh config and backup
alias zshconfig="gedit ~/.zshrc"
alias themeconfig="gedit $theme_path/powerline-custom.zsh-theme"
alias zshcustom="gedit ~/.zshrc_custom.zsh"

#### others
alias dt="cd ~/Desktop"

#### catkin
alias ckm="catkin_make"
alias ckmd="catkin_make -DCMAKE_BUILD_TYPE=Debug"
alias ckmr="catkin_make -DCMAKE_BUILD_TYPE=Release"
alias reckm="rm -r ./devel ./build && command catkin_make"
alias reckmd="rm -r ./devel ./build && command catkin_make -DCMAKE_BUILD_TYPE=Debug"
alias reckmr="rm -r ./devel ./build && command catkin_make -DCMAKE_BUILD_TYPE=Release"
######################catkin_make##########################
function ckmwp(){catkin_make -DCATKIN_WHITELIST_PACKAGES="$@"}
function ckmbp(){catkin_make -DCATKIN_BLACKLIST_PACKAGES="$@"}

#### ros
alias rp="rospack"
alias rt="rostopic"
alias rr="rosrun"
alias rl="roslaunch"
alias rpf="rospack find"
alias rtl="rostopic list"
alias rte="rostopic echo"

alias nodegraph="rosrun rqt_graph rqt_graph"
alias tftree="rosrun rqt_tf_tree rqt_tf_tree"
alias sws="source ./devel/setup.zsh"

### github
alias gitc="git clone"

### keyboard
alias keyboard_on="xinput set-prop 'AT Translated Set 2 keyboard' 'Device Enabled' 1"
alias keyboard_off="xinput set-prop 'AT Translated Set 2 keyboard' 'Device Enabled' 0"

### touchpad
alias touchpad_on="xinput set-prop 'DELL0782:00 06CB:7E92 Touchpad' 'Device Enabled' 1"
alias touchpad_off="xinput set-prop 'DELL0782:00 06CB:7E92 Touchpad' 'Device Enabled' 0"

###################### commonly used########################
#### list full path
function lfp()
{
    ls ./$@ | sed "s:^:`pwd`/:"
}

#### list folder in path
function lf()
{
    if [ -z $@ ]; then
        ls -d */
    else
        ls -F $@ |grep ".*/$"|tr -d '/'
    fi
}

#### make a folder and cd folder
function mdcd(){mkdir $@ && cd $@}

#### make a catkin work_space and cd work_space
function cwcd(){mkdir -pv $@/src && cd $@ && catkin_make}

#### make a work_space and cd work_space
function mwcd(){mkdir -pv $@/src $@/build $@/scripts $@/data $@/include && cd $@ && touch CMakeLists.txt}

#### roscd and print dir
function rcd()
{
    roscd $@ &&
    pwd
}

#### roscd packages' src folder and print dir
function rcdsrc()
{
    roscd $@ && cd src &&
    pwd
}

######################zsh concerned########################
#### zsh files backup 
function zshbackup()
{
    cp ~/.zshrc ~/.zshrc.backup && 
    echo $backuptime >> ~/.zshrc.backup
}

function themebackup()
{
    cp $theme_path/powerline-custom.zsh-theme $theme_path/powerline-custom.zsh-theme.backup &&
    echo $backuptime >> $theme_path/powerline-custom.zsh-theme.backup
}

function custombackup()
{
    cp ~/.zshrc_custom.zsh ~/.zshrc_custom.zsh.backup
    echo $backuptime >> ~/.zshrc_custom.zsh.backup
}


######################ROS functions########################
#### get ros source command form a path. eg: getsourcecmd or getsourcecmd wsname
function getsourcecmd()
{
    if [ -z $@ ]; then
        pwd | sed 's/$/\/devel\/setup.zsh --extend/' |sed 's/^/source /'
    else
        echo $@ | sed 's/$/\/devel\/setup.zsh --extend/' |sed 's/^/source /'
    fi
}

#### list ros packages dependencies. eg: pkgdep or pkgdep packagename
function pkgdep()
{
    if [ -z $@ ]; then
        rospack depends1 ${PWD##*/} |tr "\n" " "
    else
        rospack depends1 $@ |tr "\n" " "
    fi
}

#### cd ros packages' workspace. eg: cdpkgws or cdpkgws packagename
function pkgws()
{
    if [ -z $@ ]; then
        cd ../.. && echo -n '=>' && pwd
    else
        roscd $@/../.. && echo -n '=>' && pwd
    fi
}

#### list ros packages' name in a workspace. eg: listwspkg or listwspkg packagename
function wspkg()
{
    if [ -z $@ ]; then
        lf ./src
    else
        lf $workspace_ros_path/$@/src
    fi
}

#### cd workspace. eg cdws or cdws wsname
function cdws()
{
    if [ -z $@ ]; then
        cd $workspace_ros_path/$current_ws && echo -n '=>' && pwd
    else
        cd $workspace_ros_path/$@ && echo -n '=>' && pwd
    fi
}

#### cd launch folder of ros packages. eg: cdlaunch or cdlaunch packagename
function cdlaunch()
{
    if [ -z $@ ]; then
        cd ./launch && echo -n '=>' && pwd
    else
        roscd $@/launch && echo -n '=>' && pwd
    fi
}

#### list include files in a launch file. eg: llfile launchfilename1 lfn2 ...
function llfile()
{
    for i in $@
    do
    echo -e "$i contains files:\n" && cat $i |grep '<include file=".*\.launch"' | sed -e 's:.*<.*find ::' -e 's:)/launch/:\t--->\t:' -e 's:".*>::' && echo
    done
}

#### list contained nodes in a launch file. eg: llnode launchfilename1 lfn2 ...
function llnode()
{
    for i in $@
    do
    echo -e "$i contains nodes:\n" && cat $i |grep '<node.*>' | sed -e 's:.*<node *::' -e 's:/*>$::' && echo
    done
}

#### list contained arguments in a launch file. eg: llarg launchfilename1 lfn2 ...
function llarg()
{
    for i in $@
    do
    echo -e "$i contains args:\n" && cat $i |grep '<arg.*>' | sed -e 's:.*<arg *::' -e 's:/*>$::' -e 's::\t:' && echo
    done
}

#### print files, nodes and arguments contained in a launch file. eg: linfo launchfilename1 lfn2 ...
function linfo()
{
    for i in $@
    do
    echo $split; llfile $i; llnode $i; llarg $i;
    done
}

#### list ros workspaces within a customed folder.
function listws(){lf $workspace_ros_path}

#### print environmental variables line by line.
function listenv(){echo $@ | tr ":" "\n"}

#### auto source ros setup.zsh files of all ros workspaces within a customed folder
function rosautosource()
{
    source /opt/ros/kinetic/setup.zsh --extend
    eval "$(cd $workspace_ros_path && lf | sed "s:^:`pwd`/:" |sed 's/$/\/devel\/setup.zsh --extend/g' |sed 's/^/source '/)"
}

######################git functions########################

#### simpliy git push procedures
function gitpush()
{
    if [ -z $@ ]; then
        git add . && git commit -a -m "modify file" && git push origin master
    else
        git add . && git commit -a -m $@ && git push origin master
    fi
    
}

#### save zshrc and theme files to git
function zshtogit()
{
    echo $pwd
    cp ~/.zshrc $zsh_git_path/.zshrc &&
    cp $theme_path/powerline-custom.zsh-theme $zsh_git_path/powerline-custom.zsh-theme &&
    cp ~/.zshrc_custom.zsh $zsh_git_path/.zshrc_custom.zsh &&
    echo "file backup to $zsh_git_path" &&
    cd $zsh_git_path && echo "uploading files to git repository" && gitpush "modify"
}

#### modify git folder config file
function gitremoteconfig()
{
    sed -i 's|url = https://github.com/Zhang-Hongda|url = git@github.com:Zhang-Hongda|' ./.git/config
}


############################run at start#############################
#### file backup
zshbackup
themebackup
custombackup
#### activate ros workspaces
rosautosource
#############################Python path#############################
# export PYTHONPATH=/usr/lib/python3/dist-packages

