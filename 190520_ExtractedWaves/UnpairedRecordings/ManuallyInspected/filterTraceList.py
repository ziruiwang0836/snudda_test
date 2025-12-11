import os.path
import argparse

parser = argparse.ArgumentParser(description="Filters a CSV file")
parser.add_argument("infile",help="Input csv file")
parser.add_argument("--outfile",help="Output csv file")
parser.add_argument("--require","-require","-r",help="Item must include")
parser.add_argument("--exclude","-exclude","-e",help="Item must not include")

args = parser.parse_args()

inFile = args.infile

if(args.outfile):
  outFile = args.outfile
elif(args.require or args.exclude):
  outFile = inFile
  if(args.require):
    outFile += "-require-" + args.require
  if(args.exclude):
    outFile += "-exclude-" + args.exclude
else:
  print("No outfile, no require and no exclude, nothing to do!")
  exit(-1)
    
assert inFile != outFile, "Input and output files must differ"

assert os.path.isfile(inFile), "Input file does not exist"
assert not os.path.isfile(outFile), "Output file already exists"

print("Reading " + inFile)

with open(inFile,'r') as inF:
  content = inF.readlines()

  if(content[0][0] == "!"):
    # Special character indicating the file, preserve
    dataFileLine = content.pop(0)
  else:
    dataFileLine = None
  
  setList = [c.strip() for c in content]

  setList = [c.split(",") for c in setList]
  

  if(args.require is not None):
    print("Filtering, require: " + args.require)
    setList = [[x for x in c if args.require in x] for c in setList]

  if(args.exclude is not None):
    print("Filtering, exclude: " + args.exclude)
    setList = [[x for x in c if args.exclude not in x] for c in setList]

  # Filter out empty sub-lists
  setList = [c for c in setList if len(c) > 0]


setListStr = [",".join(c) for c in setList]

print("Writing " + outFile)

with open(outFile,'w') as outF:
  if(dataFileLine is not None):
    outF.write(dataFileLine)
  for str in setListStr:
    outF.write(str + "\n")

