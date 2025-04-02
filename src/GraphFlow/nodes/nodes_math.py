#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from NodeGraphQt import BaseNode
import math

__all__ = ['NumberNode', 'AddNode','MultiplyNode', 'PrintNode', 'SubNode', 'DivideNode', 'PowerNode', 'SqrtNode',
           'AbsNode','SinNode', 'CosNode']

class NumberNode(BaseNode):
    """数字节点，提供固定数值"""
    __identifier__ = 'nodes.math'
    NODE_NAME = 'Number float'

    def __init__(self):
        super(NumberNode, self).__init__()
        self.add_output('value')
        self.add_text_input('number', '', '1.0')

    def get_value(self):
        return float(self.get_property('number'))

class AddNode(BaseNode):
    """加法节点，执行两个数的加法"""
    __identifier__ = 'nodes.math'
    NODE_NAME = 'Add'

    def __init__(self):
        super(AddNode, self).__init__()
        self.add_input('input_1')
        self.add_input('input_2')
        self.add_output('output')

    def execute(self):
        # 获取输入值
        input1 = self.input(0)
        input2 = self.input(1)

        val1 = input1.connected_ports()[0].node().get_value() if input1.connected_ports() else 0
        val2 = input2.connected_ports()[0].node().get_value() if input2.connected_ports() else 0

        result = val1 + val2
        print(f"{self.name()}: {val1} + {val2} = {result}")
        return result

    def get_value(self):
        return self.execute()

class SubNode(BaseNode):
    """加法节点，执行两个数的加法"""
    __identifier__ = 'nodes.math'
    NODE_NAME = 'Sub'

    def __init__(self):
        super(SubNode, self).__init__()
        self.add_input('input_1')
        self.add_input('input_2')
        self.add_output('output')

    def execute(self):
        # 获取输入值
        input1 = self.input(0)
        input2 = self.input(1)

        val1 = input1.connected_ports()[0].node().get_value() if input1.connected_ports() else 0
        val2 = input2.connected_ports()[0].node().get_value() if input2.connected_ports() else 0

        result = val1 - val2
        print(f"{self.name()}: {val1} + {val2} = {result}")
        return result

    def get_value(self):
        return self.execute()

class MultiplyNode(BaseNode):
    """乘法节点，执行两个数的乘法"""
    __identifier__ = 'nodes.math'
    NODE_NAME = 'Multiply'

    def __init__(self):
        super(MultiplyNode, self).__init__()
        self.add_input('input_1')
        self.add_input('input_2')
        self.add_output('output')

    def execute(self):
        input1 = self.input(0)
        input2 = self.input(1)

        val1 = input1.connected_ports()[0].node().get_value() if input1.connected_ports() else 1
        val2 = input2.connected_ports()[0].node().get_value() if input2.connected_ports() else 1

        result = val1 * val2
        print(f"{self.name()}: {val1} × {val2} = {result}")
        return result

    def get_value(self):
        return self.execute()

class DivideNode(BaseNode):
    """乘法节点，执行两个数的乘法"""
    __identifier__ = 'nodes.math'
    NODE_NAME = 'Divide'

    def __init__(self):
        super(DivideNode, self).__init__()
        self.add_input('input_1')
        self.add_input('input_2')
        self.add_output('output')

    def execute(self):
        input1 = self.input(0)
        input2 = self.input(1)

        val1 = input1.connected_ports()[0].node().get_value() if input1.connected_ports() else 1
        val2 = input2.connected_ports()[0].node().get_value() if input2.connected_ports() else 1

        result = val1 / val2
        print(f"{self.name()}: {val1} × {val2} = {result}")
        return result

    def get_value(self):
        return self.execute()

class PrintNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = 'nodes.math'
    NODE_NAME = 'Print'

    def __init__(self):
        super(PrintNode, self).__init__()
        self.add_input('input')
        self.value = None

    def execute(self):
        input_port = self.input(0)
        if input_port.connected_ports():
            self.value = input_port.connected_ports()[0].node().execute()
            print(f"{self.name()} Output result: {self.value}")
    def print_value(self):
        return f"{self.name()} Output result: {self.value}"

class PowerNode(BaseNode):
    """幂运算节点，执行数的幂运算"""
    __identifier__ = 'nodes.math'
    NODE_NAME = 'Power'

    def __init__(self):
        super(PowerNode, self).__init__()
        self.add_input('base')
        self.add_input('exponent')
        self.add_output('output')

    def execute(self):
        base = self.input(0)
        exp = self.input(1)

        val_base = base.connected_ports()[0].node().get_value() if base.connected_ports() else 1
        val_exp = exp.connected_ports()[0].node().get_value() if exp.connected_ports() else 1

        result = val_base ** val_exp
        print(f"{self.name()}: {val_base} ^ {val_exp} = {result}")
        return result

    def get_value(self):
        return self.execute()

class SqrtNode(BaseNode):
    """平方根节点，计算数的平方根"""
    __identifier__ = 'nodes.math'
    NODE_NAME = 'Square Root'

    def __init__(self):
        super(SqrtNode, self).__init__()
        self.add_input('input')
        self.add_output('output')

    def execute(self):
        input_port = self.input(0)
        val = input_port.connected_ports()[0].node().get_value() if input_port.connected_ports() else 0

        if val < 0:
            raise ValueError("负数没有实数平方根")

        result = val ** 0.5
        print(f"{self.name()}: √{val} = {result}")
        return result

    def get_value(self):
        return self.execute()

class AbsNode(BaseNode):
    """绝对值节点，计算数的绝对值"""
    __identifier__ = 'nodes.math'
    NODE_NAME = 'Absolute Value'

    def __init__(self):
        super(AbsNode, self).__init__()
        self.add_input('input')
        self.add_output('output')

    def execute(self):
        input_port = self.input(0)
        val = input_port.connected_ports()[0].node().get_value() if input_port.connected_ports() else 0

        result = abs(val)
        print(f"{self.name()}: |{val}| = {result}")
        return result

    def get_value(self):
        return self.execute()

class SinNode(BaseNode):
    """正弦函数节点"""
    __identifier__ = 'nodes.trig'
    NODE_NAME = 'Sine'

    def __init__(self):
        super(SinNode, self).__init__()
        self.add_input('angle (rad)')
        self.add_output('output')

    def execute(self):
        input_port = self.input(0)
        val = input_port.connected_ports()[0].node().get_value() if input_port.connected_ports() else 0

        result = math.sin(val)
        print(f"{self.name()}: sin({val}) = {result}")
        return result

    def get_value(self):
        return self.execute()

class CosNode(BaseNode):
    """余弦函数节点"""
    __identifier__ = 'nodes.trig'
    NODE_NAME = 'Cosine'

    def __init__(self):
        super(CosNode, self).__init__()
        self.add_input('angle (rad)')
        self.add_output('output')

    def execute(self):
        input_port = self.input(0)
        val = input_port.connected_ports()[0].node().get_value() if input_port.connected_ports() else 0

        result = math.cos(val)
        print(f"{self.name()}: cos({val}) = {result}")
        return result

    def get_value(self):
        return self.execute()