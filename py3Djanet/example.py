#!/usr/bin/env python
# coding: utf-8

from packer import Packer
from methods import Item
from bin import Bin

packer = Packer()

packer.add_bin(Bin('small-envelope',  11.5,  6.125,  0.25, 10  ))
packer.add_bin(Bin('large-envelope',  15.0,   12.0,  0.75, 15  ))
packer.add_bin(Bin('small-box',      8.625,  5.375, 1.625, 70.0))
packer.add_bin(Bin('medium-box',      11.0,    8.5,   5.5, 70.0))
packer.add_bin(Bin('medium-2-box',  13.625, 11.875, 3.375, 70.0))
packer.add_bin(Bin('large-box',       12.0,   12.0,   5.5, 70.0))
packer.add_bin(Bin('large-2-box',  23.6875,  11.75,   3.0, 70.0))

packer.add_item(Item( '50g [powder 1]',  3.9370, 1.9685, 1.9685, 1))
packer.add_item(Item( '50g [powder 2]',  3.9370, 1.9685, 1.9685, 2))
packer.add_item(Item( '50g [powder 3]',  3.9370, 1.9685, 1.9685, 3))
packer.add_item(Item( '50g [powder 4]',  3.9370, 1.9685, 1.9685, 3))
packer.add_item(Item( '50g [powder 5]',  3.9370, 1.9685, 1.9685, 3))
packer.add_item(Item( '50g [powder 6]',  3.9370, 1.9685, 1.9685, 3))
packer.add_item(Item( '50g [powder 7]',  5.1240, 1.1350, 1.5435, 3))
packer.add_item(Item('250g [powder 8]',  7.8740, 3.9370, 1.9685, 4))
packer.add_item(Item('250g [powder 9]',  7.8740, 3.9370, 1.9685, 5))
packer.add_item(Item('250g [powder 10]', 7.8740, 3.9370, 1.9685, 6))
packer.add_item(Item('250g [powder 11]', 7.8740, 3.9370, 1.9685, 7))
packer.add_item(Item('250g [powder 12]', 7.8740, 3.9370, 1.9685, 8))
packer.add_item(Item('250g [powder 13]', 7.8740, 3.9370, 1.9685, 9))

packer.pack()

