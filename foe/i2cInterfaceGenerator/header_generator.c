#include "header_generator.h"
#include "utils.h"

/******************************************************************************/
/*                                Defines                                     */
/******************************************************************************/
#define HEADER_HEADER "#ifndef I2C_INTERFACE_ADDR_H\n#define I2C_INTERFACE_ADDR_H\n\n"
#define HEADER_FOOTER "\n#endif /* I2C_INTERFACE_ADDR_H */\n"
#define LOCATION_HEADER "/************** %s **************/\n"

/******************************************************************************/
/*                           Static functions                                 */
/******************************************************************************/
static void write_entry_content(FILE *file, interface_element_t *element, modifier_t modifier) {
    int address;
    fprintf(file, "#define ");

    write_addr(element->category, element->c_name, file, modifier);

    switch (modifier) {
        case NORMAL:
            address = element->address;
            break;
        case LOW:
            address = element->address;
            break;
        case HIGH:
            address = element->address + 2;
            break;
    }
    fprintf(file, " 0x%02xU\n", address);
}

static void write_entry(FILE *file, interface_element_t *element) {
    /* Check input parameters */
    if (file == NULL) {
        return;
    } else if (element == NULL) {
        return;
    }

    fprintf(file, "// %s\n", element->name);

    if (element->size < 32) {
        write_entry_content(file, element, NORMAL);
    } else {
        write_entry_content(file, element, LOW);
        write_entry_content(file, element, HIGH);
    }

    fprintf(file, "\n");

}

static void write_location_header(FILE *file, location_t location) {
    switch (location) {
    case LOCATION_FLASH:
        fprintf(file, LOCATION_HEADER, "FLASH");
        break;
    case LOCATION_RAM:
        fprintf(file, LOCATION_HEADER, "RAM");
        break;
    case LOCATION_COMMAND:
        fprintf(file, LOCATION_HEADER, "COMMAND");
        break;
    default:
        break;
    }
}

/******************************************************************************/
/*                           Public functions                                 */
/******************************************************************************/
FILE* init_header_file(char *file_name) {
    FILE *file;
    int ret;

    /* Check input parameters */
    if (file_name == NULL) {
        return NULL;
    }

    file = fopen(file_name, "w");
    if (file == NULL) {
        return NULL;
    }

    ret = fprintf(file, HEADER_HEADER);
    if (ret < 0) {
        printf("Error when writing header in %s", file_name);
    }

    return file;
}

int write_header_file(FILE *file, interface_element_t *entry, device_t *device) {
    interface_element_t *cur_entry;
    /* Check input parameters */
    if (file == NULL) {
        return -1;
    } else if (entry == NULL) {
        return -2;
    }

    if (device) {
        fprintf(file, "#define DEVICE_ADDR 0x%02XU\n\n", device->addr);
    }

    for (location_t location = 0; location < LOCATION_END; location++) {
        write_location_header(file, location);
        cur_entry = entry;
        while (cur_entry != NULL) {
            if (cur_entry->location == location) {
                write_entry(file, cur_entry);
            }

            cur_entry = cur_entry->next;
        }
    }

    return 0;
}

void close_header_file(FILE *file) {
    /* Check input parameters */
    if (file == NULL) {
        return;
    }

    fprintf(file, HEADER_FOOTER);
    fclose(file);
}
