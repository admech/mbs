import sys
import re

class Reader:
  def __init__(self, path):
    self.state = 'INITIAL'
    self.current = { 'name': 'NONAME', 'lines': [] }
    self.path = path

  def set_state(self, new_state):
    self.state = new_state
    self.current = { 'name': 'NONAME', 'lines': [] }

  def get_class_name(self, line):
    words = line.split()
    for word in words:
      if word[0] in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ':
        return word
    raise Exception('could not find class name in:\n>>>>>' + line + '<<<<<')

  def get_data(self):
    header = []
    classes = []
    with open(self.path, 'r') as fp:
      try:
        for line_num, line in enumerate(fp):
          line = line[:-1]
          if self.state == 'INITIAL':
            if line.startswith(' ') or line.startswith('import'):
              header.append(line)
            else:
              self.set_state('CLASS')
          if self.state == 'EXPECT':
            if not (line.startswith(' ') or len(line) == 0):
              name = self.get_class_name(line)
              self.set_state('CLASS')
              self.current['name'] = name
              self.current['lines'].append(line)
          elif self.state == 'CLASS':
            self.current['lines'].append(line)
            if line.startswith('end'):
              classes.append(self.current)
              self.set_state('EXPECT')
      except Exception as e:
        raise Exception('failed to read ' + self.path + ' on line ' + str(line_num) + ': ' + str(e))
    return header, classes

def render(header, classes):
  print('HEADER')
  print('-------------------------------------------------------')
  print('-------------------------------------------------------')
  print('-------------------------------------------------------')
  print('-------------------------------------------------------')
  print('-------------------------------------------------------')
  for l in header:
      print(l)
  print()
  print('CLASSES')
  print('-------------------------------------------------------')
  print('-------------------------------------------------------')
  print('-------------------------------------------------------')
  print('-------------------------------------------------------')
  print('-------------------------------------------------------')
  for c in classes:
    print('-------------------------------------------------------')
    print(c['name'])
    print('-------------------------------------------------------')
    for l in c['lines']:
      print(l)

def write_lines_to_file(path, lines):
  with open(path, 'w') as f:
    for l in lines:
      f.write(l + '\n')

def write_files(parent_package_name, package_name, read_file_path, write_dir_path, header, classes):
  package_mo = [
    'within ' + parent_package_name + ';', '',
    'package ' + package_name, '',
  ]
  for l in header:
    package_mo.append('  ' + l)
  package_mo.append('')
  package_mo.append('end ' + package_name + ';')
  write_lines_to_file(write_dir_path + '/package.mo', package_mo)

  for c in classes:
    lines = [
      'within ' + parent_package_name + ("" if len(parent_package_name) == 0 else ".") + package_name + ';', '',
    ]
    for l in c['lines']:
      lines.append(l)
    write_lines_to_file(write_dir_path + '/' + c['name'] + '.mo', lines)

def main(parent_package_name, package_name, read_file_path, write_dir_path, dry_run = True):
  reader = Reader(read_file_path)
  header, classes = reader.get_data()
  if dry_run:
    render(header, classes)
  else:
    write_files(parent_package_name, package_name, read_file_path, write_dir_path, header, classes)

if __name__ == "__main__":
  parent_package_name = sys.argv[1]
  package_name = sys.argv[2]
  read_file_path = sys.argv[3]
  write_dir_path = sys.argv[4]
  dry_run = sys.argv[-1] == '--dry'
  main(parent_package_name, package_name, read_file_path, write_dir_path, dry_run)



