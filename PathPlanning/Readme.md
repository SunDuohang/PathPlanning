# Readme.txt

使用valgrind软件，测试内存泄露

```shell
valgrind --tool=memcheck --log-file=log.txt --leak-check=full 可执行程序
```

