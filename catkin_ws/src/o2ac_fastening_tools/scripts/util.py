import yaml

####################
### ObjectConfig ###
####################


def read_object_yaml_config(conf_filename):
    f = open(conf_filename, "r")
    conf = f.read()
    data = yaml.safe_load(conf)
    f.close()

    return data
