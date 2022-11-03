

def make_sure(func): 
    def wrapper():
        while 1:
            res = input(f'are you sure you want to execute "{func.__name__}"? [y/n]')
            if res=='n':
                return
            elif res=='y':
                func()
                return
    return wrapper

@make_sure
def hello():
  print('Hello world')

if __name__=="__main__":

    hello()