all: df pa rj

df:
	grep -o -w id ../data/train/df-$(id)/cvrp-$(id)-df-$(f).json | wc -l

pa:
	grep -o -w id ../data/train/pa-$(id)/cvrp-$(id)-pa-$(f).json | wc -l

rj:
	grep -o -w id ../data/train/rj-$(id)/cvrp-$(id)-rj-$(f).json | wc -l
