

#api_key = 'apk-14166daf4872c6537ffc7e1b30afacdfc519606b7eefd932c5ad4ab5d9693f480b1a254e31ff6781937a14acc1fa4e7410d0460299a4ec3d2681fe801f79f1f81912b3826ca0cfe0c596eeb683ec32eb'
api_key = 'apk-110d39e19561e2e1538f8cb18288c7543ccfbd5f6d2881f2708c64401d0f620a302f360f2123e2b9f867f56a7c5c4479a5d07c540b32d77172c5f572d8488192878aa2be3e15dd73407656a9a2d7dbf5'
from rxn4chemistry import RXN4ChemistryWrapper

import time

rxn4chemistry_wrapper = RXN4ChemistryWrapper(api_key=api_key)
#rxn4chemistry_wrapper.list_all_projects()
#rxn4chemistry_wrapper.set_project('5f9b3e2a0c3e4f0001d8e4e4')
#print(rxn4chemistry_wrapper.list_all_projects()['response']['payload']['content'][0]['name'])
projects = rxn4chemistry_wrapper.list_all_projects()['response']['payload']['content']
time.sleep(1)
for project in projects:
    if project['name'] == 'MAPs':
        rxn4chemistry_wrapper.set_project(project['id'])
    print(project['name'])

print(rxn4chemistry_wrapper.project_id)

time.sleep(1)
response = rxn4chemistry_wrapper.predict_automatic_retrosynthesis(
    'CCO'
)
time.sleep(1)



print(response)