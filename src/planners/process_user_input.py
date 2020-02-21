from nltk.corpus import wordnet
import yaml


class ProcessInput:

    def __init__(self, dictFile):
        self.file = dictFile
        self.exactDict = set()
        self.matchDict = set()
        self.stop_words = set(
            ['the', 'is', 'and', 'a', 'like', 'i', 'that', 'didn\'t'])

    def buildDicts(self):
        with open(self.file) as file:
            file_data = yaml.load(file, Loader=yaml.FullLoader)
            self.exactDict = set(file_data.items()[0][1])
            self.matchDict = set(file_data.items()[1][1])

    # check similarity to words in dictionaries
    def getSimScores(self, word):
        ret = {}
        # check if word matches any in dictionary
        if word in self.exactDict:
            ret[word] = 1
            for k in self.exactDict:
                if k != word:
                    ret[k] = 0
        # if word doesnt match generate similarity scores
        else:
            for k in self.matchDict:
                comp = wordnet.synset(k)
                # generate a score for each definition of word
                scores = [syn.wup_similarity(comp) for syn in wordnet.synsets(
                    word) if syn.wup_similarity(comp) is not None]
                # take the max score if it exists
                ret[k.split('.')] = max(scores) if scores else 0
        return ret

    def processUserInput(self, string):
        # convert to list removing stopwords
        words = [word for word in string.split()
                 if word not in self.stop_words]
        sims = {}
        # itterate through words and generate similarities
        for word in words:
            # generate similarity scores
            sims[word] = self.getSimScores(word)
        return sims


if __name__ == "__main__":
    pi = ProcessInput('../../config/dictionaries.yml')
    pi.buildDicts()
    print(pi.processUserInput(
        'i didn\'t like that the robot moved the cup over the laptop'))
