// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: BSD-3-Clause

// Tests of loading vocabularies from text files.

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include "FORB.h"
#include "TemplatedVocabulary.h"

namespace {

using Vocabulary =
    DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>;

int failures = 0;

void check(bool condition, const std::string& what)
{
    if (!condition) {
        std::cerr << "FAIL: " << what << '\n';
        ++failures;
    }
}

// Returns the text of a descriptor whose bytes all hold the value.
std::string descriptor(int value)
{
    std::string result;

    for (int i = 0; i < DBoW2::FORB::L; ++i) {
        result += std::to_string(value) + ' ';
    }

    return result;
}

// A vocabulary that does not exist leaves nothing to read, whatever values
// the header check would compare.
void rejectsMissingFile()
{
    Vocabulary vocabulary;

    check(!vocabulary.loadFromTextFile("missing/vocabulary.txt"),
          "a missing vocabulary is rejected");
}

// A header that ends before its last value leaves the values that the
// vocabulary held before.
void rejectsIncompleteHeader(const std::string& header)
{
    const std::string path = "incomplete.txt";
    {
        std::ofstream out{path};
        out << header;
    }

    Vocabulary vocabulary;

    check(!vocabulary.loadFromTextFile(path),
          "the header \"" + header + "\" is rejected");
}

// A vocabulary of two words below the root, as saveToTextFile writes it.
void loadsWordsOfFile()
{
    const std::string path = "two-words.txt";
    {
        std::ofstream out{path};
        // Two branches, one level, L1 norm scoring and TF-IDF weighting.
        out << "2 1 0 0\n";
        out << "0 1 " << descriptor(0) << "1\n";
        out << "0 1 " << descriptor(255) << "1\n";
    }

    Vocabulary vocabulary;

    check(vocabulary.loadFromTextFile(path), "the vocabulary loads");
    check(vocabulary.size() == 2, "the vocabulary holds two words");
}

// Returns the number of lines of the file.
int lines(const std::string& path)
{
    std::ifstream in{path};
    int result = 0;

    for (std::string line; std::getline(in, line);) {
        ++result;
    }

    return result;
}

// Saving a loaded vocabulary writes the header and the nodes it loaded, no
// more.
void savesNodesItLoads()
{
    const std::string path = "loaded.txt";
    const std::string saved = "saved.txt";
    {
        std::ofstream out{path};
        out << "2 1 0 0\n";
        out << "0 1 " << descriptor(0) << "1\n";
        out << "0 1 " << descriptor(255) << "1\n";
    }

    Vocabulary vocabulary;
    check(vocabulary.loadFromTextFile(path), "the vocabulary loads");
    vocabulary.saveToTextFile(saved);

    check(lines(saved) == lines(path),
          "the saved vocabulary has " + std::to_string(lines(path)) +
              " lines, got " + std::to_string(lines(saved)));
}

} // namespace

int main()
{
    rejectsMissingFile();
    rejectsIncompleteHeader("");
    rejectsIncompleteHeader("10 5 0\n");
    loadsWordsOfFile();
    savesNodesItLoads();

    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
