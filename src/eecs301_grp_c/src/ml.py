import sys
import csv

def main():
  args = sys.argv[1:]
  try:
    filename = args[0]
    print filename
    csv_file = open(filename, 'wb')
    wr = csv.writer(csv_file)
    wr.writerow(['hello'])
  except IndexError:
    print "Please enter valid arguments."

if __name__ == "__main__": main()