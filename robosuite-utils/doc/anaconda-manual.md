# Anaconda manual

아나콘다 설치 및 사용방법 정리

### 아나콘다 설치

- [주소](https://www.anaconda.com/products/individual)로 들어가 다운로드

- 다운받은 위치로 가서 실행 파일로 변경 후 실행

  ~~~
  $ chmod +x Anaconda3-2021.05-Linux-x86_64.sh
  $ ./Anaconda3-2021.05-Linux-x86_64.sh
  ~~~

### 아나콘다 가상환경 생성

* 가상환경 이름은 mytest로 파이썬 버전은 3.6으로 설치

  ~~~
  $ conda create -n mytest python=3.6
  ~~~

* requirements.txt 사용

  ~~~
  $ conda create -n mytest --file requirements.txt python=3.6
  ~~~

### 가상환경 활성화

~~~
$ conda activate
$ conda activate mytest
~~~

### 가상환경 비활성화

~~~
$ conda deactivate
~~~

### 아나콘다 패키지 설치

~~~
$ conda install pkg
~~~

### 가상환경 삭제

~~~
$ conda env remove -n mytest
~~~

### 가상환경 이름 변경

~~~
$ conda create --name new_name --clone old_name
$ conda remove --name old_name --all
~~~

