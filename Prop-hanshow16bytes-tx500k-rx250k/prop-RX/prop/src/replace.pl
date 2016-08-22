
##
# This file is used to replace 903 char to 908, including file name, folder name and file content
##

use strict;
use warnings;
use File::Find ;

&main();

sub main()
{
    my $file = 'proprietary.c';

    # Read
    open(FILE , '<' , $file ) or die "Can not open $file: $!\n" ;
    my @readFile = <FILE>;
    close FILE;
    
    # Replace
    foreach(@readFile )   {   s/PROP_.*_()/908/g;  };
    
    # Write
    open(FILE , '>' , $file ) or die "Can not open $file: $!\n" ;
    print FILE @oldFile ;
    close FILE;
}
