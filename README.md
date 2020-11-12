# ToDo
Navigate to where you want the project to be

`$ cd /paht/to/project/`


Clone the repo to where you want it

`$ git clone https://github.com/jorgenmyrvold/robotteknikk-mini-project.git`


Change dir into project

`cd robotteknikk-mini-project`

Install python virtualenv. Make sure that you are in the `robotteknikk-mini-project` directory

`Python3 -m venv .`


Activate venv (virtual environment). `<venv>` is the directory to the venv file. The dir where you ran the last command -^

`source <venv>/bin/activate`

You should now se in the terminal something like `(robotteknikk-mini-project) <the usual stuff>`


Install packages

`pip3 install -r requirements.txt`


check the installation by running

```
$ python3
>>> import modern_robotics as mr
>>> print(help(mr.FKinBody))
```