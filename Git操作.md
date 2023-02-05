# Git操作基础

---

- @author: ShiZhong Tan

- @date: 2023-02-05

---



>隔了好久没用，就有的不太记得了，就饿那么几个操作与其每次都搜索找，不如自己写下来记着了

### 基础操作

配置用户基本信息
```
git config --global user.name '你的用户名'
git config --global user.email '你的邮箱'
```

从远程克隆服务器
`git clone <url> [directory]`
url为git仓库地址，directory为本地目录

从远程仓库拉取代码
`git pull`
git pull命令用于从另一个存储库或本地分支获取并集成(整合)，在默认模式下，git pull是git fetch后跟git merge FETCH_HEAD的缩写，使用格式：
`git pull [options] [<repository> [<refspec>…]]`

`git fetch`
`git merge`
git fetch可以提取远程仓库的数据，如果有多个远程仓库，我们可以在后面加仓库的别名。该命令执行完后需要执行git merge 远程分支到你所在的分支。

提交本地代码
```
git add .
git commit -m '本次提交信息'
git push origin main
```

对修改文件操作
```
暂存文件的命令：git add <文件名>
放弃未暂存文件的修改命令：git checkout – <文件名>
将被修改的文件暂存并提交的命令：git commit -a
将被修改的文件暂存并添加注释：git commit -m '本次提交信息'
```

初始化本地仓库
`git init`

查看当前文件状态
`git status`

列出分支
`git branch`

创建分支
`git branch (branchname)`

切换分支
`git checkout (branchname)`

删除分支
`git branch -d (branchname)`

合并分支(将任意分支合并到到当前分支中)
`git merge branchname`

创建新分支并立即切换到该分支
`创建新分支并立即切换到该分支下`

取消已缓存的内容
`git reset HEAD test.txt`

查看提交历史
`git log`

```
git log	//查看提交历史记录，从最近到最远，可以看到3次
git log --pretty=oneline	//加参，简洁查看
git reflog	//查看每一次修改历史
cat test.txt	//查看文件内容
git status	//查看工作区中文件当前状态
git reset --hard HEAD^（HEAD~100）（commit id）	//回退版本
git checkout -- test.txt	//丢弃工作区的修改，即撤销修改
git reset HEAD test.txt	//丢弃暂存区的修改（若已提交，则回退）
```

### 远程仓库
```
ssh-keygen -t rsa -C "youremail@example.com"	//创建SSH Key
git remote add origin git@github.com:Daisy/AKgit.git	//关联
git push -u origin master	//将本地内容推送到远程仓库（第一次）
git push origin master	//将本地内容推送到远程仓库（之后）
git remote -v        //查看远程仓库信息
git remote rm origin	//删除远程仓库（解绑）
git clone git@github.com: Daisy/AKgit.git	//克隆远程仓库
//克隆之后使用和查看
cd gitskills
ls
git remote	//查看远程库的信息
git remote -v	//查看远程库的详细信息
```

### 分支管理
```
$ git checkout -b dev	//创建并切换到分支dev
//创建并切换到分支dev，同上
$ git branch dev	//创建
$ git checkout dev	//切换
//新版本
$ git switch -c dev	//创建并切换到分支dev
$ git switch master	//直接切换分支
$ git branch		//查看当前分支
$ git merge dev	（--no-ff）(-m)//合并，把dev分支的工作成果合并到master分支上
$ git branch -d dev	//删除dev分支

$ git push origin master（dev）	//推送分支
$ git checkout -b dev origin/dev	//创建远程origin的dev分支到本地
$ git pull	//抓取分支（解决冲突）
$ git branch --set-upstream-to=origin/dev dev//指定本地与远程dev的链接
$ git rebase	//把本地未push的分叉提交历史整理成直线
```

### 标签管理
```
$ git tag v1.0	//打标签
$ git tag -a v0.1 -m "version 0.1 released" 1094adb //指定标签名和说明文字
$ git tag	//查看所有标签
//若是忘记打，则查找历史提交commit id ，再打上
$ git log --pretty=oneline --abbrev-commit
$ git tag v0.9 f52c633
$ git show v0.9		//查看标签详细信息
$ git tag -d v0.1	//删除标签
$ git push origin v1.0	//推送标签到远程
$ git push origin –tags	//一次性推送全部本地标签
//删除标签，（若已推送到远程，先从本地删除，从远程删除）
$ git tag -d v0.9
$ git push origin :refs/tags/v0.9 
```