变量命名规则:
继承了匈牙利命名法的大部分特点. 这种命名方法看上去十分冗长, 但是不仅可以做到"见名知义",还可"见名知类型".
同学们自己写程序的时候还是以实现功能为第一要务, 不必遵循程序中的命名规则.
对于非数学对象, 变量名主要由: 作用域 + 类型 + 描述 组成. 如 `ExperNodeBase.hpp` 中的 `ExperNodeBase::mupNodeHandle`:
 - m:   member, 作用域, 表示这个变量是类的成员变量
 - up:  unique_ptr, 类型, 表示这个变量是 std::unique_ptr 型变量
 - NodeHandle: 描述, 遵循驼峰命名法, 表示这个变量和节点句柄相关

作用域一般有这几类:
 - m: member, 类的成员变量
 - l: local,  程序中定义的局部变量, 但是一般这个前缀不加
 - g: global, 全局变量
 
常用的类型有这几类:
基本类型:
 - n: number, int, unsigned int, long, uint64_t, size_t 等等表示整数的变量
 - c: 特指 char, unsigned char
 - f: float型浮点数
 - d: double 型浮点数
 - e: enum, 枚举变量
 - p: 指针, 通常与上述搭配, 如 pdMax 表示指向最大值的指针变量, 指向的变量是 double 类型
标准库类型:
 - str: string, 字符串, 通常指代 std::string
 - v:   vector, 通常指代 std::vector
 - map: map,    键值对映射表, 通常指代 std::map 一族
 - list:list,   列表, 通常指代 std::list
 - a:   array,  通常指代 boost::array 和 std::array,; 传统 C 风格数组使用 p 表示
 - sp:  shared_ptr, 共享指针, 通常指代 boost::shared_ptr 和 std::shared_ptr
 - up:  unique_ptr, 不知道咋说, 通常指代 boost::unique_ptr 和 std::unique_ptr

方便起见,对于指针和带有模板参数的基本类型和标准库类型可以"套娃":
```C++

std::vector<size_t> vnVar1;
std::unique_ptr<std::vector<float> > upvfVar2;
std::vector<std::vector<int*> >      vvpnVar3;
std::vector<std::vector<std::vector<float> > >
                                     vvvfVar3;
```


