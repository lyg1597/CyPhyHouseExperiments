from apps.follow import BasicFollowApp as AppClass
from multirun import run


def main(argv):
    run(argv[1], AppClass)


if __name__ == "__main__":
    import sys
    main(sys.argv)
