#include <stdio.h>
#include <string.h>

#include "interface_parser.h"
#include "doc_generator.h"
#include "header_generator.h"
#include "interface_generator.h"

#define XML_INPUT "XML_INPUT"
#define OUTPUT_SPEC "OUTPUT_SPEC"
#define OUTPUT_HEADER "OUTPUT_HEADER"
#define OUTPUT_CODE "OUTPUT_CODE"
#define GEN_DOC "GEN_DOC"
#define GEN_HEADER "GEN_HEADER"
#define GEN_CODE "GEN_CODE"

#define MAX_VALUE_LENGTH 100

int main(int argc, char* argv[]) {
    char xml_input_file_name[MAX_VALUE_LENGTH];
    char output_spec_file_name[MAX_VALUE_LENGTH];
    char output_header_file_name[MAX_VALUE_LENGTH];
    char output_code_file_name[MAX_VALUE_LENGTH];
    int gen_doc = 0;
    int gen_header = 0;
    int gen_code = 0;
    char param_name[MAX_VALUE_LENGTH], param_value[MAX_VALUE_LENGTH];
    device_t device;

    if (argc != 2) {
        printf("Wrong number of arguments\n");
        return 1;
    }

    FILE *config_file = fopen(argv[1], "r");
    if (config_file == NULL) {
        return 1;
    }

    /* Parse config file */
    while (fscanf(config_file, "%s = %s", param_name, param_value) != EOF) {
        if (strcmp(param_name, XML_INPUT) == 0) {
            strcpy(xml_input_file_name, param_value);
        } else if (strcmp(param_name, OUTPUT_SPEC) == 0) {
            strcpy(output_spec_file_name, param_value);
        } else if (strcmp(param_name, OUTPUT_HEADER) == 0) {
            strcpy(output_header_file_name, param_value);
        } else if (strcmp(param_name, OUTPUT_CODE) == 0) {
            strcpy(output_code_file_name, param_value);
        } else if (strcmp(param_name, GEN_DOC) == 0) {
            if (strcmp(param_value, "1") == 0) {
                gen_doc = 1;
            }
        } else if (strcmp(param_name, GEN_HEADER) == 0) {
            if (strcmp(param_value, "1") == 0) {
                gen_header = 1;
            }
        } else if (strcmp(param_name, GEN_CODE) == 0) {
            if (strcmp(param_value, "1") == 0) {
                gen_code = 1;
            }
        }
    }

    xmlDoc *doc = init_parsing(xml_input_file_name);
    interface_element_t *entry = parse(doc, &device);

if (gen_doc) {
    FILE *doc_file = init_doc_file(output_spec_file_name);
    write_doc_file(doc_file, entry, &device);
    close_doc_file(doc_file);
}

if (gen_header) {
    FILE *header_file = init_header_file(output_header_file_name);
    write_header_file(header_file, entry, &device);
    close_header_file(header_file);
}

if (gen_code) {
    FILE *code_file = init_interface_file(output_code_file_name);
    write_interface_file(code_file, entry, &device);
    close_interface_file(code_file);
}

    close_parsing(doc, entry);

    return 0;
}
