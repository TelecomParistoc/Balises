#include "interface_parser.h"
#include <string.h>

static int isNumeric(char c) {
    if ((c >= '0') && (c <= '9')) {
        return 1;
    } else {
        return 0;
    }
}

static int isHexa(char c) {
    if (isNumeric(c)) {
        return 1;
    } else if ((c >= 'A') && (c <= 'F')){
        return 1;
    } else {
        return 0;
    }
}

/* Extract size from well-formed type */
static int get_size(char* type) {
    int index;
    int size = 0;
    int multiplier = 1;

    if (type == NULL) {
        return 0;
    }

    index = strlen(type) - 1;
    while (isNumeric(type[index])) {
        size += (type[index] - '0') * multiplier;
        index--;
        multiplier *= 10;
    }

    return size;
}

static int get_addr(char *addr_str) {
    int index;
    int multiplier = 1;
    int addr = 0;

    if (addr_str == NULL) {
        return 255;
    }

    index = strlen(addr_str) - 1;
    while(isHexa(addr_str[index])) {
        if (isNumeric(addr_str[index])) {
            addr += (addr_str[index] - '0') * multiplier;
        } else {
            addr += (addr_str[index] - 'A' + 10) * multiplier;
        }
        index--;
        multiplier *= 16;
    }

    return addr;
}

static void process_entry(xmlNode *entry_element, interface_element_t *entry) {
    while (entry_element != NULL) {
        if (strcmp((const char*)entry_element->name, "name") == 0) {
            snprintf(entry->name, NAME_MAX_SIZE, "%s", entry_element->children->content);
        } else if (strcmp((const char*)entry_element->name, "type") == 0) {
            snprintf(entry->type, 20, "%s", entry_element->children->content);
            entry->size = get_size(entry->type);
        } else if (strcmp((const char*)entry_element->name, "address") == 0) {
            entry->address = get_addr((char*)entry_element->children->content);
        } else if (strcmp((const char*)entry_element->name, "read") == 0) {
            if (strcmp((const char*)entry_element->children->content, "true") == 0)
                entry->read_access = 1;
        } else if (strcmp((const char*)entry_element->name, "write") == 0) {
            if (strcmp((const char*)entry_element->children->content, "true") == 0)
                entry->write_access = 1;
        } else if (strcmp((const char*)entry_element->name, "c_name") == 0) {
            snprintf(entry->c_name, NAME_MAX_SIZE, "%s", entry_element->children->content);
        } else if (strcmp((const char*)entry_element->name, "category") == 0) {
            snprintf(entry->category, CAT_MAX_SIZE, "%s", entry_element->children->content);
        } else if (strcmp((const char*)entry_element->name, "gen_code") == 0) {
            if (strcmp((const char*)entry_element->children->content, "true") == 0) {
                entry->gen_code = 1;
            } else {
                entry->gen_code = 0;
            }
        }
        entry_element = entry_element->next;
    }
}

static interface_element_t* add_entry(interface_element_t *entry) {
    if (entry == NULL) {
        entry = (interface_element_t*)malloc(sizeof(interface_element_t));
        memset(entry, 0, sizeof(interface_element_t));
    } else {
        while (entry->next != NULL) {
            entry = entry->next; /* Should never be executed */
        }
        entry->next = (interface_element_t*)malloc(sizeof(interface_element_t));
        entry = entry->next;
        memset(entry, 0, sizeof(interface_element_t));
    }

    return entry;
}

/* entry = pointer on last filled entry */
static interface_element_t* process_element(xmlNode *node, interface_element_t *entry, device_t *device) {
    xmlNode *cur_node;
    static location_t location;

    for (cur_node = node; cur_node; cur_node = cur_node->next) {
        if (cur_node->type == XML_ELEMENT_NODE) {
            if (strcmp((const char*)cur_node->name, "register") == 0) {
                entry = add_entry(entry);
                process_entry(cur_node->children, entry);
                entry->location = location;
            } else if (strcmp((const char*)cur_node->name, "registers") == 0) {
                if ((cur_node->properties != NULL) && (strcmp((const char*)cur_node->properties->name, "location") == 0)) {
                    location = LOCATION_END;
                    if (strcmp((const char*)cur_node->properties->children->content, "flash") == 0) {
                        location = LOCATION_FLASH;
                    } else if (strcmp((const char*)cur_node->properties->children->content, "ram") == 0) {
                        location = LOCATION_RAM;
                    } else if (strcmp((const char*)cur_node->properties->children->content, "command") == 0) {
                        location = LOCATION_COMMAND;
                    }

                    if (location != LOCATION_END) {
                        entry = process_element(cur_node->children, entry, device);
                    }

                }
            } else {
                if ((cur_node->properties != NULL) && (strcmp((const char*)cur_node->properties->name, "device_addr") == 0)) {
                    if (device != NULL) {
                        device->addr = get_addr((char*)cur_node->properties->children->content);
                    }
                }
                entry = process_element(cur_node->children, entry, device);
            }
        }
    }

    return entry;
}


xmlDoc* init_parsing(char *file_name) {
    LIBXML_TEST_VERSION
    xmlDoc *doc;

    /* Check input parameters */
    if (file_name == NULL) {
        printf("NULL xml file name\n");
        return NULL;
    }

    doc = xmlReadFile(file_name, NULL, 0);
    return doc;
}

interface_element_t* parse(xmlDoc* doc, device_t *device) {
    xmlNode *root_element;
    interface_element_t entry;
    entry.next = NULL;

    root_element = xmlDocGetRootElement(doc);
    process_element(root_element, &entry, device);
    return entry.next;
}

void close_parsing(xmlDoc *doc, interface_element_t *entry) {
    interface_element_t *tmp;

    if (doc == NULL) {
        return;
    }
    xmlFreeDoc(doc);
    xmlCleanupParser();

    while (entry != NULL) {
        tmp = entry->next;
        free(entry);
        entry = tmp;
    }
}
