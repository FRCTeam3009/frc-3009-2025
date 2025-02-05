def something(do_something):
    do_something()

def say():
    print("stuff")

say()
something(say)

stuff = 42
something(lambda: print(stuff))
