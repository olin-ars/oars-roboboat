# OARS Git tutorial 2016

First, you're going to set up SSH keys.

```
$ ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
```

Press enter to save to the default directory file.

Press enter twice rather than entering a passphrase.

Now, you're going to check that the ssh-agent is enabled, then add your new SSH key to the ssh-agent.
```
$ eval "$(ssh-agent -s)"
$ ssh-add ~/.ssh/id_rsa
```

In order to make copying your key easier, you're going to install xclip, then copy the contents of your id_rsa.pub file.
```
$ sudo apt-get install xclip
$ xclip -sel clip < ~/.ssh/id_rsa.pub
```
At this point, you're ready to add the key to your GitHub account.

Go to your profile settings and select **SSH and GPG keys** from the sidebar on the left.

Give your key a descriptive title and paste your key into the "Key" field.

Click the **Add SSH key** button and confirm your password if asked to.

###If you already have an SSH Key that you would rather use (instead of creating a new one), feel free to check out the links below.

This information was largely taken from GitHub's help documentation; specific pages can be found [here](https://help.github.com/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent/) and [here](https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/).

###Adding the OARS repo

If you haven't already cloned the OARS repo to your computer, do that now. Go to this year's [OARS git repo](https://github.com/olin-robotic-sailing/oars-roboboat) and click **Clone or download** on the right. Choose SSH rather than HTTPS and copy the link.

Use your terminal to navigate to the local folder into which you want to clone the repo. Once there, type
```
$ git clone 
```
and paste the SSH at the end. Press enter and you will have a copy of the git repo locally accessible on your computer.