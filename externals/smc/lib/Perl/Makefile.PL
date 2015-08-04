use ExtUtils::MakeMaker;
# See lib/ExtUtils/MakeMaker.pm for details of how to influence
# the contents of the Makefile that is written.
WriteMakefile(
    'NAME'          => 'DFA::Statemap',
    'VERSION_FROM'  => 'lib/DFA/Statemap.pm',
    'ABSTRACT'      => 'SMC runtime',
    'PREREQ_PM'     => {},
    'AUTHOR'        => 'Francois PERRAD (francois.perrad@gadz.org)',
    'dist'          => {
                        'COMPRESS'      => 'gzip',
                        'SUFFIX'        => '.gz',
    },
);

