import csv

def mean(arr):
    n = len(arr)
    total = sum(arr)
    average = total / n
    return average

def variance(arr, average):
    n = len(arr)
    variance = 0
    for i in arr:
        variance += (average - arr[i]) ** 2
    return variance / n


def FindMaxima(numbers):
  maxima = []
  length = len(numbers)

  if length >= 2:
    if numbers[0] > numbers[1]:
      maxima.append(numbers[0])
       
    if length > 3:
      for i in range(1, length-1):     
        if numbers[i] > numbers[i-1] and numbers[i] > numbers[i+1]:
          maxima.append(numbers[i])

    if numbers[length-1] > numbers[length-2]:    
      maxima.append(numbers[length-1])        
  return maxima
   

def main():
  with open('../../../data.csv', 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=' ')
    with open('../../../maxima.csv', 'a') as writefile:
      wr = csv.writer(writefile)
      for row in reader:
        line = row[0].split(',')
        shape = line[0]
        data = map(lambda x: int(x), line[1:])
        wr.writerow([shape, len(FindMaxima(data))])

if __name__ == "__main__": main()